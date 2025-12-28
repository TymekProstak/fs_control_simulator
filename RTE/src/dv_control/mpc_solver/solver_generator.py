import json
import numpy as np
import os
from casadi import SX, reshape
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

# --- Konfiguracja ścieżek (bez zmian) ---
def find_acados_paths():
    if "ACADOS_SOURCE_DIR" in os.environ:
        acados_dir = os.environ["ACADOS_SOURCE_DIR"]
    else:
        local_dir = os.path.abspath("../External/acados")
        if os.path.isdir(local_dir):
            acados_dir = local_dir
        else:
            raise RuntimeError("ACADOS not found! Set ACADOS_SOURCE_DIR.")
    return f"{acados_dir}/install/include", f"{acados_dir}/install/lib"

CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))


# zbudowanie ścieżki względnej niezależnie od miejsca uruchomienia!

JSON_PATH = os.path.join(CURRENT_PATH, "../config/Params/control_param.json")

print("Wczytuję JSON z:", JSON_PATH)


with open(JSON_PATH, "r") as f:

    cfg = json.load(f)


    # ---- wyciągamy parametry ----


    N = cfg["mpc"]["solver"]["mpc_N"] # horyzont MPC

    eps = cfg["mpc"]["solver"]["eps"] # tolerancja solvera

    maxit = cfg["mpc"]["solver"]["max_iter"] # maksymalna liczba iteracji


    # OGRANICZENIA FIZYCZNE

    delta_max = cfg["mpc"]["bounds"]["max_delta"] # max kąt skrętu [rad]

    delta_min = cfg["mpc"]["bounds"]["min_delta"] # min kąt skrętu [rad]

    ddelta_max = cfg["mpc"]["bounds"]["max_ddelta"]

    ddelta_min = cfg["mpc"]["bounds"]["min_ddelta"]



    dt = 1.0/ cfg["general"]["odom_frequency"] # krok czasowy MPC


    # Wagi kosztu

    Q_y = cfg["mpc"]["cost"]["Q_y"] # waga błędu lateralnego

    Q_psi = cfg["mpc"]["cost"]["Q_psi"] # waga błędu kąta

    R_delta = cfg["mpc"]["cost"]["Q_delta"] # waga skrętu kiery

    R_d_delta = cfg["mpc"]["cost"]["R_ddelta"] # waga zmiany sterowania 
    NX = 7 # liczba stanów [ey, epsi, vy, r, delta, d_delta, delta_request]

    NU = 1 # liczba sterowań [d_delta_request]


    print("Używam parametrów MPC:")

    print(f"N = {N}")

    print(f"dt = {dt}")

    print(f"max_ddelta = {ddelta_max}") 



def create_ltv_discrete_model():
    model = AcadosModel()
    x = SX.sym('x', NX)
    u = SX.sym('u', NU)
    
    # Parametry modelu LTV (Ad, Bd, Kd)
    np_Ad = NX * NX
    np_Bd = NX * NU
    np_Kd = NX
    NP = np_Ad + np_Bd + np_Kd

    p = SX.sym('p', NP)

    # Rozpakowanie (column-major)
    Ad_flat = p[0:np_Ad]
    Bd_flat = p[np_Ad:np_Ad+np_Bd]
    Kd      = p[np_Ad+np_Bd : np_Ad+np_Bd+np_Kd]

    Ad = reshape(Ad_flat, NX, NX)
    Bd = reshape(Bd_flat, NX, NU)

    x_next = Ad @ x + Bd @ u + Kd

    model.x = x
    model.u = u
    model.p = p
    model.disc_dyn_expr = x_next
    model.name = "mpc_ltv_discrete"

    return model, NP

def create_ocp_solver():
    model, NP = create_ltv_discrete_model()
    ocp = AcadosOcp()
    ocp.model = model

    # Wymiary
    ocp.dims.N = N
    ocp.dims.nx = NX
    ocp.dims.nu = NU
    ocp.dims.np = NP

    # ---- COST (LINEAR_LS) ----
    # To jest kluczowe dla wydajności. Pozostajemy przy LINEAR_LS.
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # Definicja macierzy Vx, Vu (mapowanie stanów na wektor błędu)
    # y = [x; u] (pełny stan + sterowanie)
    ny = NX + NU
    ny_e = NX

    ocp.dims.ny = ny
    ocp.dims.ny_e = ny_e

    ocp.cost.Vx = np.zeros((ny, NX))
    ocp.cost.Vx[:NX, :] = np.eye(NX)
    
    ocp.cost.Vu = np.zeros((ny, NU))
    ocp.cost.Vu[NX:, :] = np.eye(NU)

    ocp.cost.Vx_e = np.eye(NX)

    # --- INICJALIZACJA WAG (WARTOŚCI STARTOWE) ---
    # Te wartości zostaną "wypalone" w kodzie C jako domyślne,
    # ale będziemy je nadpisywać w Pythonie.
    
    # Macierz wag dla etapów 0..N-1
    W = np.zeros((ny, ny))
    W[0, 0]   = Q_y
    W[1, 1]   = Q_psi
    W[NX, NX] = R_d_delta # Waga sterowania
    
    ocp.cost.W = W
    ocp.cost.yref = np.zeros((ny, ))

    # Macierz wag dla etapu końcowego N
    W_e = np.zeros((ny_e, ny_e))
    W_e[0, 0] = Q_y
    W_e[1, 1] = Q_psi
    
    ocp.cost.W_e = W_e
    ocp.cost.yref_e = np.zeros((ny_e, ))

    # ---- CONSTRAINTS (bez zmian) ----
    ocp.constraints.x0 = np.zeros((NX, ))
    # ... (Twoje ograniczenia na delta, ddelta) ...
    # Dla przykładu uproszczone:
    ocp.constraints.idxbx = np.array([4], dtype=int)  # indeks stanu delta
    ocp.constraints.lbx = np.array([delta_min])
    ocp.constraints.ubx = np.array([delta_max])
    ocp.constraints.idxbu = np.array([0], dtype=int)
    ocp.constraints.lbu = np.array([ddelta_min])
    ocp.constraints.ubu = np.array([ddelta_max])

    # ---- SOLVER OPTIONS ----
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.tf = N * dt

    # Ścieżki
    inc_path, lib_path = find_acados_paths()
    ocp.code_export_directory = "./acados_solver_generated"
    ocp.acados_include_path = inc_path
    ocp.acados_lib_path = lib_path
    ocp.parameter_values = np.zeros(NP)

    # Generowanie
    json_file = "acados_ocp_ltv.json"
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    
    return solver, NP

if __name__ == "__main__":
    solver, NP = create_ocp_solver()
    print("Solver wygenerowany pomyślnie.")
    print("Możesz teraz dynamicznie zmieniać wagi W używając solver.cost_set()")