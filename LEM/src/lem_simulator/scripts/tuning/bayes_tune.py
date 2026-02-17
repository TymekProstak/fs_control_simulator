#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bayes_tune.py
=============

Bayesian Optimization do strojenia kontrolera przez symulator LEM.

Architektura (rozdzielona odpowiedzialność):
- lem_eval.py     : "czarna skrzynka" f(theta, track_id) -> {cost, crashed, metrics...}
- bayes_tune.py   : BO/GP/VDP + wybór kolejnych punktów
- track_policy.py : wybór torów (single / fixed list / random range / curriculum)

Co minimalizuję?
- Minimalizuję koszt J (liczba skalarna), liczony przez lem_eval na podstawie metryk:
  J = sum_k w_k * metric_k  (np. w_speed < 0 => większa prędkość zmniejsza koszt)
  a gdy crash:
  J = crash_penalty + crash_time_penalty*(sim_time - crash_time)

Jak BO radzi sobie z crashami / nieciągłościami?
- Do GP (modelu zastępczego) NIE wrzucam surowych kosztów z crashy, bo to jest nieciągłe.
- Uczę GP na sukcesach, a crash-e dodaję jako VDP (Virtual Data Points):
  dla punktów crash generuję "wirtualną obserwację" y_hat = max(mu, y_best) + beta*sigma
  gdzie (mu,sigma) to predykcja GP z sukcesów. To daje gładkie zniechęcenie do regionów infeasible.

Jak wybieram następne theta?
- GP daje predykcję (mu, sigma) dla punktu u w [0,1]^d
- Minimalizuję akwizycję LCB(u) = mu(u) - kappa * sigma(u)
  (duże kappa -> eksploracja, małe -> eksploatacja)

Uwaga dot. zależności od plików:
- wymaga track_policy.py w tym samym katalogu
- wymaga lem_eval.py w tym samym katalogu
"""

import os
import sys
import json
import time
import math
import random
import argparse
import subprocess
from typing import Dict, Any, List, Tuple, Optional

from track_policy import TrackPolicy


# -------------------------
# Small utilities
# -------------------------

def expand(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))

def median(xs: List[float]) -> float:
    ys = sorted(xs)
    n = len(ys)
    if n == 0:
        return float("inf")
    if n % 2 == 1:
        return ys[n // 2]
    return 0.5 * (ys[n // 2 - 1] + ys[n // 2])

def ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)


# -------------------------
# Parameter space transforms
# -------------------------

class ParamSpace:
    """
    Optymalizuję po wektorze u in [0,1]^d.
    Potem mapuję na theta (prawdziwe wartości do JSON).

    Dla wag MPC (Q_y, Q_psi, R_ddelta) używam skali log,
    bo zakresy są szerokie i GP zwykle stabilniej działa w log-przestrzeni.
    """
    def __init__(self, params_cfg: Dict[str, Any]):
        self.keys: List[str] = list(params_cfg.keys())
        self.cfg = params_cfg

        self.lo: List[float] = []
        self.hi: List[float] = []
        self.scale: List[str] = []
        self.init: List[float] = []
        self.aliases: Dict[str, List[str]] = {}

        for k in self.keys:
            c = params_cfg[k]
            self.lo.append(float(c["lo"]))
            self.hi.append(float(c["hi"]))
            self.scale.append(str(c.get("scale", "linear")))
            self.init.append(float(c.get("init", 0.5 * (float(c["lo"]) + float(c["hi"])))))
            self.aliases[k] = list(c.get("aliases", []))

    def dim(self) -> int:
        return len(self.keys)

    def clamp01(self, u: List[float]) -> List[float]:
        return [max(0.0, min(1.0, float(x))) for x in u]

    def u_to_theta(self, u: List[float]) -> Dict[str, float]:
        u = self.clamp01(u)
        theta: Dict[str, float] = {}
        for i, k in enumerate(self.keys):
            lo, hi = self.lo[i], self.hi[i]
            if self.scale[i] == "log":
                # log-scale requires positive bounds
                if lo <= 0 or hi <= 0:
                    x = lo + (hi - lo) * u[i]
                else:
                    a = math.log(lo)
                    b = math.log(hi)
                    x = math.exp(a + (b - a) * u[i])
            else:
                x = lo + (hi - lo) * u[i]

            theta[k] = float(x)

            # jeśli w projekcie/README jest literówka w ścieżce, dajemy alias (kompatybilność)
            for ak in self.aliases.get(k, []):
                theta[ak] = float(x)

        return theta

    def theta_to_u(self, theta: Dict[str, float]) -> List[float]:
        u: List[float] = []
        for i, k in enumerate(self.keys):
            lo, hi = self.lo[i], self.hi[i]
            x = theta.get(k, None)
            if x is None:
                for ak in self.aliases.get(k, []):
                    if ak in theta:
                        x = theta[ak]
                        break
            if x is None:
                x = self.init[i]

            x = float(x)
            if self.scale[i] == "log" and lo > 0 and hi > 0:
                a = math.log(lo)
                b = math.log(hi)
                xx = max(lo, min(hi, x))
                uu = (math.log(xx) - a) / (b - a) if b > a else 0.5
            else:
                uu = (max(lo, min(hi, x)) - lo) / (hi - lo) if hi > lo else 0.5
            u.append(max(0.0, min(1.0, uu)))
        return u


# -------------------------
# Gaussian Process Regression (minimal, no numpy)
# -------------------------

def matern52_kernel(x: List[float], y: List[float], ell: List[float], sigma_f: float) -> float:
    s = 0.0
    for i in range(len(x)):
        d = (x[i] - y[i]) / max(1e-12, ell[i])
        s += d * d
    r = math.sqrt(5.0 * s)
    return sigma_f * sigma_f * (1.0 + r + (r * r) / 3.0) * math.exp(-r)

def se_kernel(x: List[float], y: List[float], ell: List[float], sigma_f: float) -> float:
    s = 0.0
    for i in range(len(x)):
        d = (x[i] - y[i]) / max(1e-12, ell[i])
        s += d * d
    return sigma_f * sigma_f * math.exp(-0.5 * s)

def cholesky_decomp(A: List[List[float]]) -> Optional[List[List[float]]]:
    """
    Zwraca L takie że A = L L^T, albo None jeśli się nie da (numerycznie).
    """
    n = len(A)
    L = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = A[i][j]
            for k in range(j):
                s -= L[i][k] * L[j][k]
            if i == j:
                if s <= 1e-14:
                    return None
                L[i][j] = math.sqrt(s)
            else:
                L[i][j] = s / max(1e-12, L[j][j])
    return L

def chol_solve(L: List[List[float]], b: List[float]) -> List[float]:
    """
    Rozwiązuje (L L^T) x = b.
    """
    n = len(L)
    y = [0.0] * n
    for i in range(n):
        s = b[i]
        for k in range(i):
            s -= L[i][k] * y[k]
        y[i] = s / max(1e-12, L[i][i])

    x = [0.0] * n
    for i in reversed(range(n)):
        s = y[i]
        for k in range(i + 1, n):
            s -= L[k][i] * x[k]
        x[i] = s / max(1e-12, L[i][i])
    return x

class GaussianProcess:
    """
    Minimalny GP: fit(X,y) i predict(x) -> (mu, var)
    """
    def __init__(self, kernel_name: str, noise: float, dim: int):
        self.kernel_name = kernel_name
        self.noise = float(noise)
        self.dim = dim

        self.X: List[List[float]] = []
        self.y: List[float] = []

        self.ell: List[float] = [0.2] * dim
        self.sigma_f: float = 1.0

        self.L: Optional[List[List[float]]] = None
        self.alpha: Optional[List[float]] = None

    def kernel(self, x: List[float], y: List[float]) -> float:
        if self.kernel_name == "se":
            return se_kernel(x, y, self.ell, self.sigma_f)
        return matern52_kernel(x, y, self.ell, self.sigma_f)

    def set_hyperparams(self, ell: List[float], sigma_f: float) -> None:
        self.ell = [max(1e-3, float(e)) for e in ell]
        self.sigma_f = max(1e-6, float(sigma_f))

    def fit(self, X: List[List[float]], y: List[float]) -> bool:
        self.X = [list(map(float, xi)) for xi in X]
        self.y = list(map(float, y))
        n = len(self.X)
        if n == 0:
            self.L = None
            self.alpha = None
            return False

        K = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i + 1):
                kij = self.kernel(self.X[i], self.X[j])
                K[i][j] = kij
                K[j][i] = kij
        for i in range(n):
            K[i][i] += self.noise

        L = cholesky_decomp(K)
        if L is None:
            return False

        alpha = chol_solve(L, self.y)
        self.L = L
        self.alpha = alpha
        return True

    def predict(self, x: List[float]) -> Tuple[float, float]:
        if self.L is None or self.alpha is None or len(self.X) == 0:
            return 0.0, 1.0

        n = len(self.X)
        kvec = [self.kernel(self.X[i], x) for i in range(n)]
        mu = sum(kvec[i] * self.alpha[i] for i in range(n))

        v = chol_solve(self.L, kvec)
        kxx = self.kernel(x, x) + self.noise
        var = max(1e-12, kxx - sum(kvec[i] * v[i] for i in range(n)))
        return mu, var


# -------------------------
# Acquisition optimization
# -------------------------

def lcb(mu: float, sigma: float, kappa: float) -> float:
    return mu - kappa * sigma

def pattern_refine(u0: List[float], score_fn, steps: int, step0: float = 0.15) -> List[float]:
    """
    Prosta lokalna poprawa bez gradientów:
    - startuję z u0
    - próbuję +/- krok w każdej osi
    - akceptuję jeśli akwizycja spada
    - gdy brak poprawy: zmniejszam krok
    """
    u = u0[:]
    best = score_fn(u)
    step = step0
    d = len(u)

    for _ in range(steps):
        improved = False
        for i in range(d):
            for sgn in (+1.0, -1.0):
                cand = u[:]
                cand[i] = max(0.0, min(1.0, cand[i] + sgn * step))
                val = score_fn(cand)
                if val < best:
                    best = val
                    u = cand
                    improved = True
        if not improved:
            step *= 0.85
            if step < 1e-3:
                break
    return u


# -------------------------
# Calling lem_eval
# -------------------------

def run_lem_eval(lem_eval_path: str, config_path: str, theta: Dict[str, float], track_id: int) -> Dict[str, Any]:
    """
    Wołam lem_eval jako osobny proces.
    Zwraca JSON (albo "crashed" jeśli coś poszło nie tak).
    """
    cmd = [
        sys.executable, lem_eval_path,
        "--config", config_path,
        "--theta_json", json.dumps(theta),
        "--track_id", str(track_id),
        "--print_json_only"
    ]
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if p.returncode != 0:
        return {
            "crashed": True,
            "cost": float("inf"),
            "metrics": {"crashed": 1, "crash_reason": "lem_eval_failed", "stderr": p.stderr[-400:]},
            "track_id": track_id
        }
    try:
        return json.loads(p.stdout.strip())
    except Exception:
        return {
            "crashed": True,
            "cost": float("inf"),
            "metrics": {"crashed": 1, "crash_reason": "lem_eval_bad_json"},
            "track_id": track_id
        }


# -------------------------
# Virtual Data Points (VDP) for crashes
# -------------------------

def _var(xs: List[float]) -> float:
    if len(xs) < 2:
        return 1.0
    m = sum(xs) / len(xs)
    return sum((x - m) * (x - m) for x in xs) / (len(xs) - 1)

def build_vdp_dataset(
    X_succ: List[List[float]], y_succ: List[float],
    X_crash: List[List[float]],
    gp_cfg: Dict[str, Any],
    beta: float,
    clip_to_success_range: bool
) -> Tuple[List[List[float]], List[float]]:
    """
    Buduję zbiór do trenowania GP, który "gładko" uwzględnia crash-e:
    - trenuję pomocniczy GP na sukcesach
    - dla każdego crasha dodaję wirtualną obserwację y_hat = max(mu, y_best) + beta*sigma
    """
    if len(X_succ) == 0:
        return X_succ[:], y_succ[:]

    dim = len(X_succ[0])
    gp = GaussianProcess(gp_cfg.get("kernel", "matern52"), float(gp_cfg.get("noise", 1e-4)), dim)

    gp.set_hyperparams([0.25] * dim, sigma_f=max(1e-6, math.sqrt(max(1e-12, _var(y_succ)))))

    ok = gp.fit(X_succ, y_succ)
    if not ok:
        return X_succ[:], y_succ[:]

    y_best = min(y_succ)
    y_min = min(y_succ)
    y_max = max(y_succ)

    X_aug = X_succ[:]
    y_aug = y_succ[:]

    for xc in X_crash:
        mu, var = gp.predict(xc)
        sig = math.sqrt(max(1e-12, var))
        y_hat = max(mu, y_best) + float(beta) * sig

        if clip_to_success_range:
            y_hat = min(max(y_hat, y_min), y_max)

        X_aug.append(xc)
        y_aug.append(float(y_hat))

    return X_aug, y_aug


# -------------------------
# Aggregation
# -------------------------

def aggregate_cost(costs: List[float], mode: str) -> float:
    if mode == "mean":
        return sum(costs) / max(1, len(costs))
    return median(costs)


# -------------------------
# Initial design (LHS)
# -------------------------

def lhs_samples(n: int, d: int) -> List[List[float]]:
    """
    Prosty Latin Hypercube Sampling w [0,1]^d bez numpy.
    """
    out: List[List[float]] = []
    bins = []
    for j in range(d):
        perm = list(range(n))
        random.shuffle(perm)
        bins.append(perm)

    for i in range(n):
        u = []
        for j in range(d):
            b = bins[j][i]
            lo = b / n
            hi = (b + 1) / n
            u.append(lo + (hi - lo) * random.random())
        out.append(u)
    return out


# -------------------------
# Main BO loop
# -------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="tuning_config.json")
    ap.add_argument("--out", default="", help="output dir (default: .../results/run_YYYYmmdd_HHMMSS)")
    args = ap.parse_args()

    cfg_path = expand(args.config)
    cfg = json.load(open(cfg_path, "r"))

    # locate scripts (assume same directory)
    this_dir = os.path.dirname(os.path.abspath(__file__))
    lem_eval_path = os.path.join(this_dir, "lem_eval.py")
    if not os.path.exists(lem_eval_path):
        print(f"[FATAL] Missing lem_eval.py at: {lem_eval_path}", file=sys.stderr)
        return 2

    # output dir
    if args.out.strip():
        out_dir = expand(args.out)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(this_dir, "results", f"run_{ts}")
    ensure_dir(out_dir)

    # seed
    seed = int(cfg.get("bo", {}).get("seed", 0))
    random.seed(seed)

    # separate RNG for track sampling (so track randomness is reproducible independently)
    tracks_rng = random.Random(seed + 12345)

    # space
    space = ParamSpace(cfg["params"])
    d = space.dim()

    # track policy (single/random/fixed/curriculum)
    track_policy = TrackPolicy.from_cfg(cfg)

    # eval config
    eval_cfg = cfg.get("eval", {})
    repeats_per_track = int(eval_cfg.get("repeats_per_track", 1))
    agg_mode = str(eval_cfg.get("aggregation", "median"))

    # when treat whole theta as "crashed" (for GP/VDP dataset split)
    crash_rate_as_crash = float(eval_cfg.get("crash_rate_as_crash", 0.0))
    # optional: soft penalty for crash risk (encourages stability)
    crash_risk_weight = float(eval_cfg.get("crash_risk_weight", 0.0))

    bo_cfg = cfg["bo"]
    gp_cfg = cfg["gp"]

    n_init = int(bo_cfg.get("n_init", max(d + 1, 10)))
    n_iter = int(bo_cfg.get("n_iter", 50))

    kappa0 = float(bo_cfg.get("kappa_start", 3.0))
    kappa1 = float(bo_cfg.get("kappa_end", 0.7))

    rand_prob = float(bo_cfg.get("random_prob", 0.1))
    acq_samples = int(bo_cfg.get("acq_num_random_samples", 2000))
    refine_steps = int(bo_cfg.get("acq_local_refine_steps", 150))

    vdp_enabled = bool(bo_cfg.get("vdp_enabled", True))
    vdp_beta = float(bo_cfg.get("vdp_beta", 3.0))
    vdp_clip = bool(bo_cfg.get("vdp_clip_to_success_range", True))

    # data buffers
    X_all: List[List[float]] = []
    y_all: List[float] = []
    crashed_all: List[bool] = []

    # best snapshots:
    # - best_safe: tylko gdy crashed_theta == False
    # - best_any : minimalne J niezależnie od crash (żeby nie kończyć z Infinity)
    best_safe_y = float("inf")
    best_safe_u: Optional[List[float]] = None
    best_safe_theta: Optional[Dict[str, float]] = None

    best_any_y = float("inf")
    best_any_u: Optional[List[float]] = None
    best_any_theta: Optional[Dict[str, float]] = None

    history_path = os.path.join(out_dir, "history.jsonl")

    def log_record(rec: Dict[str, Any]) -> None:
        with open(history_path, "a") as f:
            f.write(json.dumps(rec) + "\n")

    # ---- helper: evaluate one u ----
    def evaluate_u(u: List[float], iter_idx: int, tag: str) -> Tuple[float, bool, Dict[str, Any]]:
        """
        Oceniam theta = u_to_theta(u) przez:
        - wybór torów: track_policy.pick_tracks(...)
        - powtórzenia: repeats_per_track dla każdego toru
        - agregacja: median/mean na kosztach epizodów

        Zwracam:
        - J (agregowany koszt)
        - crashed_theta (bool) -> używane do rozdziału sukces/crash w GP
        - rec (log)
        """
        theta = space.u_to_theta(u)

        tracks = track_policy.pick_tracks(eval_index=iter_idx, tag=tag, rng=tracks_rng)

        per_episode_costs: List[float] = []
        per_episode_crashed: List[bool] = []
        per_episode_meta = []

        for tid in tracks:
            for rep in range(repeats_per_track):
                out = run_lem_eval(lem_eval_path, cfg_path, theta, tid)
                c = float(out.get("cost", float("inf")))
                cr = bool(out.get("crashed", False))
                per_episode_costs.append(c)
                per_episode_crashed.append(cr)
                per_episode_meta.append({
                    "track_id": tid,
                    "rep": rep,
                    "crashed": cr,
                    "cost": c,
                    "crash_reason": out.get("metrics", {}).get("crash_reason", "")
                })

        n_eps = max(1, len(per_episode_costs))
        crash_count = sum(1 for x in per_episode_crashed if x)
        crash_rate = crash_count / n_eps

        J = aggregate_cost(per_episode_costs, agg_mode)

        # soft risk penalty (optional): prefer stability even if median doesn't "see" rare crashes
        if crash_risk_weight > 0.0:
            J = float(J) + crash_risk_weight * crash_rate

        # define "crashed theta" for GP split
        if crash_rate_as_crash <= 0.0:
            crashed_theta = (crash_rate > 0.0)
        else:
            crashed_theta = (crash_rate >= crash_rate_as_crash)

        rec = {
            "iter": int(iter_idx),
            "tag": tag,
            "u": [float(x) for x in u],
            "theta": {k: float(v) for k, v in theta.items()},

            "tracks": tracks,
            "repeats_per_track": int(repeats_per_track),

            "episodes": per_episode_meta,
            "crash_count": int(crash_count),
            "crash_rate": float(crash_rate),

            "J_agg": float(J),
            "timestamp_unix": float(time.time())
        }
        log_record(rec)

        return float(J), bool(crashed_theta), rec

    def update_bests(J: float, cr: bool, u: List[float]) -> None:
        nonlocal best_any_y, best_any_u, best_any_theta
        nonlocal best_safe_y, best_safe_u, best_safe_theta

        # best_any always
        if J < best_any_y:
            best_any_y = float(J)
            best_any_u = u[:]
            best_any_theta = space.u_to_theta(u)

        # best_safe only if not crashed
        if (not cr) and (J < best_safe_y):
            best_safe_y = float(J)
            best_safe_u = u[:]
            best_safe_theta = space.u_to_theta(u)

    def save_best_snapshot() -> None:
        snap = {
            "best_safe_cost": best_safe_y,
            "best_safe_theta": best_safe_theta,
            "best_safe_u": best_safe_u,
            "best_any_cost": best_any_y,
            "best_any_theta": best_any_theta,
            "best_any_u": best_any_u,
        }
        with open(os.path.join(out_dir, "best_theta.json"), "w") as f:
            json.dump(snap, f, indent=2)

    # ---- initial design (LHS) ----
    init_points = lhs_samples(n_init, d)
    for i, u in enumerate(init_points, start=1):
        J, cr, _ = evaluate_u(u, i, tag="init")
        X_all.append(u)
        y_all.append(J)
        crashed_all.append(cr)
        update_bests(J, cr, u)
        save_best_snapshot()

    # ---- BO loop ----
    for it in range(n_init + 1, n_init + n_iter + 1):
        # schedule kappa (exploration -> exploitation)
        t = (it - (n_init + 1)) / max(1, (n_iter - 1))
        kappa = (1.0 - t) * kappa0 + t * kappa1

        # split dataset
        X_succ = [X_all[i] for i in range(len(X_all)) if not crashed_all[i]]
        y_succ = [y_all[i] for i in range(len(y_all)) if not crashed_all[i]]
        X_cr = [X_all[i] for i in range(len(X_all)) if crashed_all[i]]

        # if no successes yet, fall back to random (BO can't model anything yet)
        if len(X_succ) < max(2, d):
            cand_u = [random.random() for _ in range(d)]
            J, cr, _ = evaluate_u(cand_u, it, tag="random_fallback")
            X_all.append(cand_u)
            y_all.append(J)
            crashed_all.append(cr)
            update_bests(J, cr, cand_u)
            save_best_snapshot()
            continue

        # VDP augmentation for crashes
        if vdp_enabled and len(X_cr) > 0:
            X_fit, y_fit = build_vdp_dataset(X_succ, y_succ, X_cr, gp_cfg, vdp_beta, vdp_clip)
        else:
            X_fit, y_fit = X_succ, y_succ

        # fit GP with random-restart hyperparams
        best_fit_gp = None
        best_fit_score = float("inf")

        for _ in range(int(gp_cfg.get("hyperparam_random_restarts", 8))):
            gp = GaussianProcess(gp_cfg.get("kernel", "matern52"), float(gp_cfg.get("noise", 1e-4)), d)

            ell = [10 ** random.uniform(math.log10(0.08), math.log10(0.7)) for _ in range(d)]
            sigma_f = max(1e-6, math.sqrt(max(1e-12, _var(y_fit))))
            gp.set_hyperparams(ell, sigma_f)

            ok = gp.fit(X_fit, y_fit)
            if not ok:
                continue

            # crude fit score: avg predictive variance on training points (smaller -> more confident)
            score = 0.0
            for xi in X_fit:
                _, var = gp.predict(xi)
                score += var
            score /= max(1, len(X_fit))

            if score < best_fit_score:
                best_fit_score = score
                best_fit_gp = gp

        if best_fit_gp is None:
            # fallback random if GP fails numerically
            cand_u = [random.random() for _ in range(d)]
            J, cr, _ = evaluate_u(cand_u, it, tag="random_gp_fail")
            X_all.append(cand_u)
            y_all.append(J)
            crashed_all.append(cr)
            update_bests(J, cr, cand_u)
            save_best_snapshot()
            continue

        gp = best_fit_gp

        # epsilon-random exploration
        if random.random() < rand_prob:
            cand_u = [random.random() for _ in range(d)]
            J, cr, _ = evaluate_u(cand_u, it, tag="epsilon_random")
            X_all.append(cand_u)
            y_all.append(J)
            crashed_all.append(cr)
            update_bests(J, cr, cand_u)
            save_best_snapshot()
            continue

        # choose next point by minimizing acquisition (LCB)
        def acq_score(u: List[float]) -> float:
            mu, var = gp.predict(u)
            sig = math.sqrt(max(1e-12, var))
            return lcb(mu, sig, kappa)

        # random search for good seeds
        best_cands: List[Tuple[float, List[float]]] = []
        for _ in range(acq_samples):
            uu = [random.random() for _ in range(d)]
            sc = acq_score(uu)
            best_cands.append((sc, uu))
        best_cands.sort(key=lambda t: t[0])

        seed_u = best_cands[0][1]
        refined = pattern_refine(seed_u, acq_score, steps=refine_steps, step0=0.15)

        J, cr, _ = evaluate_u(refined, it, tag="bo_step")
        X_all.append(refined)
        y_all.append(J)
        crashed_all.append(cr)
        update_bests(J, cr, refined)
        save_best_snapshot()

    # final write
    summary = {
        "best_safe_cost": best_safe_y,
        "best_safe_theta": best_safe_theta,
        "best_safe_u": best_safe_u,

        "best_any_cost": best_any_y,
        "best_any_theta": best_any_theta,
        "best_any_u": best_any_u,

        "n_evals": len(X_all),
        "out_dir": out_dir
    }
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print("\n=== DONE (Bayes) ===")
    print(f"out_dir: {out_dir}")
    print(f"best_safe_cost: {best_safe_y}")
    print(f"best_any_cost : {best_any_y}")
    if best_safe_theta:
        print("best_safe_theta:")
        for k, v in best_safe_theta.items():
            print(f"  {k}: {v:.6g}")
    elif best_any_theta:
        print("best_any_theta (no safe success found):")
        for k, v in best_any_theta.items():
            print(f"  {k}: {v:.6g}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())