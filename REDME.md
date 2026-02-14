# Symulacja DV – Planning + SLAM + Kontrola (LEM)

Na razie robimy symualcje tylko dla LEM : czyli pojazd z napędem na tył i w wersji bez kontroli trakcji,
wpływ lov level kontrolerów/napędu na tył będziemy sprawdzać gdy maszynki nie będe crashowały co drugi raz

Używamy ROS1 Noetic ważne że Noetic, bo te dystrubucje niekoniecznią są kompatybilne

## Konfiguracja środowiska ()

Abyś mógł wogole używać poleceń typu catkin build, to musisz zaincajlizwać w folderze LEM
Aby uruchomić którąkolwiek z nich, po zbudowaniu pakietów (poleceniem `catkin build` w odpowiednim folderze), należy załadować (`source`) odpowiednią konfigurację środowiska ROS.

W pliku `~/.bashrc` dopisz jedną z poniższych linii (zależnie od używanego wariantu):

Dla **LEM**:
```bash
source sciezka_do_repo/LEM/devel/setup.bash
```


> **Uwaga:** Aby uniknąć konfliktów w środowisku ROS, upewnij się, że w pliku `~/.bashrc` zakomentowane są inne setupy (poza głównym `opt/ros/noetic/setup.bash`), pozostawiając aktywny tylko ten dla LEM lub RTE.

---

## Argumenty uruchomieniowe (Launch Options)

Symulacja posiada trzy główne opcje konfiguracyjne:

| Argument | Wartość domyślna | Opis |
| :--- | :--- | :--- |
| `ins_mode` | `"gauss"` | Określa tryb symulacji DRI.<br>- `kalman`: symulacja filtra Kalmana karmionego IMU i GPS RTK.<br>- `gauss`: Ground Truth + szum biały + bias. |
| `sim_time` | `"-1"` | Czas trwania symulacji w sekundach.<br>- `-1`: nieskończoność.<br>- `int`: liczba sekund. |
| `low_level_controllers` | `"false"` | Określa, czy na `dv_board` włączone są algorytmy niskopoziomowe.<br>- **LEM**: włącza kontrolę trakcji (TC).<br>- **RTE**: włącza TC + Torque Vectoring + optymalną alokację momentu (uwzględniającą nacisk na koła). |

> **Wskazówka:** Dla wiernego odwzorowania symulacji LEM, parametr `low_level_controllers` powinien pozostać ustawiony na `"false"`.

---

## Uruchamianie symulacji

Całość można uruchomić (np. korzystając z **Foxglove**), wykonując poniższe polecenia w podanej kolejności w oddzielnych terminalach:

```bash
# 1. Start rdzenia ROS
roscore



# 2. Uruchomienie symulatora (z opcjonalnymi argumentami) - na razie w wersji którą testujemy
roslaunch lem_simulator sim.launch ins_mode:="kalman" low_level_controllers:="false" sim_time:="500" 

to znaczy że czas symulacji każdej ustawiamy na 500 sek ( chyba że skraszuje)


# 3. Uruchomienie kontroli
# UWAGA: Przed uruchomieniem upewnij się, że zmienne środowiskowe dla acados są wyeksportowane (LD_LIBRARY_PATH),
# aby linker nie zgłaszał błędów.
# DLA LEM :  LD_LIBRARY_PATH=sciezka_do_repo/LEM/src/dv_control/External/acados/install/lib:$LD_LIBRARY_PATH 
rosrun dv_control dv_control_node
```

---

## Konfiguracja zależności MPC (acados)

Aby skonfigurować niezbędne zależności dla kontrolera MPC, wykonaj poniższe kroki:

1. **Wejdź do katalogu solvera:**
   ```bash
   cd dv_control/mpc_solver
   ```

2. **Nadaj uprawnienia i zainstaluj acados lokalnie:**
   ```bash
   chmod +x install_acados_local.sh
   chmod +x run_solver_gen.sh
   ./install_acados_local.sh
   ```

3. **Ręczna instalacja `t_renderer`:**
   Automatyczne pobieranie może nie działać na starszych wersjach Ubuntu.
   * Pobierz plik ręcznie: [t_renderer v0.0.31](https://github.com/acados/tera_renderer/releases/tag/v0.0.31)
   * Skopiuj pobrany plik do: `dv_control/External/acados/bin`
   * Zmień nazwę pliku na: `t_renderer`
   * Nadaj prawa do uruchamiania:
       ```bash
       chmod +x dv_control/External/acados/bin/t_renderer
       ```

4. **Wygeneruj solver:**
   ```bash
   cd dv_control/mpc_solver
   ./run_solver_gen.sh
   ```

Jeżeli proces przebiegł poprawnie, logi w konsoli poinformują o pomyślnym wygenerowaniu solvera.

---

## Uwagi (MPC)

Zmiana parametru `mpc_horizon` lub dowolnych parametrów z kategorii `bounds` w plikach konfiguracyjnych (np. zmiana horyzontu ze 100 na inną wartość) **wymaga ponownej kompilacji solvera**.

Po każdej takiej zmianie należy uruchomić:
```bash
./run_solver_gen.sh
```

Parametry które strojimy :
adres jsona :     fs_control_simulator/LEM/src/dv_control/config/Params/control_param.json
nazwy kluczy : mpc.cost.Q_y
	       mpc.cost.Q_psi
	       mpc.cost.R_ddelta
	       
inne rzeczy w cost możesz dać na zero
ponadto strojimy jeszce : velocity_planner.v_max
			   velocity_planner.max_accel
			   velocity_planner.max_devel
			   velocity_planner.max_corrnering_accel
			   
rzeczy z metrykami jakości przejazdu pojawią sie w folderze w lems_simulator/logs/run_default_metrics.csv
ważne jest póki to czy : crashed jest 1 czy 0 ( bool)
ey_avg_m -> do fk kosztu
vs_avg_mpc -> do fk kosztu
kappa_metric -> do fk kosztu
slip_angle_metric -> do fk kosztu




aplikacja do wizaulazcji : nazywa się Foxglove możesz ją normlanie pobrać na ubuntu
aby Foxgolve widzał wiadomości co idą po odpowienich topicach to musisz zrobić :
roslaunch rosbridge_server  rosbridge_websocket.launch

jest szansa żę trzeba będzie to nie działać, i trzeab coś skonfirugrwać systemowe w bashu, czat ci powie 

mój setup paneli w foxglovie na patrzenie na info dynamiczne jest jako agh_racing_lem_json czy jakoś tak w głownym repo ( możesz go sobie importować i nie będziesz musiał tych paneli ustawiać itd)


