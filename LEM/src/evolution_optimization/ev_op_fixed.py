#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import random
import math
import signal
import subprocess
import socket
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple, List

# =========================
# USER CONFIG (EDYTUJ TUTAJ)
# =========================

EXPERIMENT_ID = "fs0"
WORKSPACE_ROOT = os.path.expanduser("~/fs_control_simulator/LEM/src")

SIM_LAUNCH = os.path.join(WORKSPACE_ROOT, "lem_simulator", "launch", "sim.launch")
CTRL_LAUNCH = os.path.join(WORKSPACE_ROOT, "dv_control", "launch", "control.launch")

CONTROL_PARAM_JSON = os.path.join(WORKSPACE_ROOT, "dv_control", "config", "Params", "control_param.json")
METRICS_CSV = os.path.join(WORKSPACE_ROOT, "lem_simulator", "logs", "run_default_metrics.csv")

DEFAULT_SIM_TIME_S = 120
ACADOS_LIB = os.path.expanduser("~/fs_control_simulator/LEM/src/dv_control/External/acados/install/lib")

TRACK_INDEX_MIN = 1
TRACK_INDEX_MAX = 50

SIM_INS_MODE = "kalman"
SIM_LOW_LEVEL_CONTROLERS = "false"

COST_WEIGHTS = {
    "vs_avg_mps": -2.0,
    "ey_avg_m": 30.0,
    "kappa_metric": 175.0,
    "slip_angle_metric": 175.0,
}
CRASH_PENALTY = 15.0
CRASH_TIME_PENALTY = 0.05

PARAM_BOUNDS = {
    "velocity_planner.v_max": (8.0, 20.0),
    "velocity_planner.max_corrnering_accel": (4.0, 20.0),
    "velocity_planner.max_accel": (4.0, 20.0),
    "velocity_planner.max_decel": (-20.0, -4.0),

    "mpc.cost.Q_y": (1.0, 10000.0),
    "mpc.cost.Q_psi": (1.0, 10000.0),
    "mpc.cost.R_ddelta": (1.0, 6000.0),
}

PARAM_INIT = {
    "velocity_planner.v_max": 12.5,
    "velocity_planner.max_corrnering_accel": 7.0,
    "velocity_planner.max_accel": 9.8,
    "velocity_planner.max_decel": -8.0,

    "mpc.cost.Q_y": 50.0,
    "mpc.cost.Q_psi": 200.0,
    "mpc.cost.R_ddelta": 600.0,
}

# =========================
# CMA parameterization
# =========================
# CMA works in an unconstrained vector space (genotype g in R^n).
# We then map g -> actual controller parameters (phenotype x).
#
# - Linear params: g = x (same units), bounds: [lo, hi]
# - Log params:    g = ln(x), bounds: [ln(lo), ln(hi)]
#   This makes CMA steps additive in log-space => multiplicative in x-space.
#
# Sigma conventions:
# - Linear params:  initial 1σ ~= (hi - lo) / 3
# - Log params:     initial 1σ in log-space = 0.1 * ln(hi/lo)
#                  => typical multiplicative factor ~= exp(±0.1 ln(hi/lo)) = (hi/lo)^0.1

LOG_PARAMS = {
    "mpc.cost.Q_y",
    "mpc.cost.Q_psi",
    "mpc.cost.R_ddelta",
}

LOG_SIGMA_FRAC = 0.10
LIN_SIGMA_FRAC = 1.0 / 3.0

# Keep a deterministic parameter order everywhere (CMA vectors <-> dict).
PARAM_KEYS = list(PARAM_BOUNDS.keys())

# ===== CMA config =====
CMA_LAMBDA = 10

# ===== Evaluation policy =====
EPISODES_PER_CANDIDATE = 3          # dokładnie 3 przejazdy na kandydata
TOTAL_EPISODES_BUDGET = 1024       # twardo

# max gen tak, żeby większość budżetu poszła na CMA, a resztę dobijemy na końcu
# episodes/gen = popsize * EPISODES_PER_CANDIDATE
EPISODES_PER_GEN = CMA_LAMBDA * EPISODES_PER_CANDIDATE
CMA_MAXGEN = TOTAL_EPISODES_BUDGET // EPISODES_PER_GEN  # dla 10*3 => 34 (1020 ep.)
EXTRA_EPISODES = TOTAL_EPISODES_BUDGET - CMA_MAXGEN * EPISODES_PER_GEN  # 4

METRICS_WAIT_TIMEOUT_S = 10.0
EPISODE_HARD_TIMEOUT_MARGIN_S = 30.0

# ===== ROS master =====
ROS_MASTER_HOST = "127.0.0.1"
ROS_MASTER_PORT = 11311
START_ROSCORE_IF_NEEDED = True

# ===== ROS isolation =====
ROS_HOME = os.path.expanduser(f"~/.ros_{EXPERIMENT_ID}")
ROS_LOG_DIR = os.path.join(ROS_HOME, "log")

# ===== SEED + CHECKPOINTING =====
TRAIN_SEED = 123
LOG_EVERY_EPISODES = 50
CHECKPOINT_LOG_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__) or "."), "cma_checkpoints.jsonl")

# =========================
#  RUNTIME GLOBALS
# =========================
EPISODES_DONE = 0
NEXT_LOG_AT = LOG_EVERY_EPISODES
BEST_COST_SO_FAR = float("inf")
BEST_THETA_SO_FAR: Dict[str, float] = {}

# =========================
#  IMPLEMENTATION
# =========================

def _must_exist(path: str, label: str) -> None:
    if not os.path.exists(path):
        raise FileNotFoundError(f"{label} does not exist: {path}")

def _ensure_ros_dirs() -> None:
    os.makedirs(ROS_HOME, exist_ok=True)
    os.makedirs(ROS_LOG_DIR, exist_ok=True)

def _ros_env(extra: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    _ensure_ros_dirs()
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = f"http://{ROS_MASTER_HOST}:{ROS_MASTER_PORT}"
    env["ROS_IP"] = "127.0.0.1"
    env["ROS_HOME"] = ROS_HOME
    env["ROS_LOG_DIR"] = ROS_LOG_DIR
    if extra:
        env.update(extra)
    return env

def _rosmaster_is_up(host: str = ROS_MASTER_HOST, port: int = ROS_MASTER_PORT, timeout: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False

def _popen_roscore() -> subprocess.Popen:
    cmd = ["roscore", "-p", str(ROS_MASTER_PORT)]
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        env=_ros_env(),
        preexec_fn=os.setsid,
        text=True,
        bufsize=1,
        universal_newlines=True,
    )
    p._cmd = cmd
    return p

def _set_json_path(d: Dict[str, Any], path: str, value: Any) -> None:
    keys = path.split(".")
    cur = d
    for k in keys[:-1]:
        if k not in cur or not isinstance(cur[k], dict):
            cur[k] = {}
        cur = cur[k]
    cur[keys[-1]] = value

def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return json.load(f)

def _save_json_atomic(path: str, data: Dict[str, Any]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)

def _parse_metrics_csv(path: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    with open(path, "r") as f:
        lines = f.read().splitlines()
    if not lines:
        return out
    for line in lines[1:]:
        if not line.strip() or "," not in line:
            continue
        k, v = line.split(",", 1)
        k = k.strip()
        v = v.strip()
        if len(v) >= 2 and v[0] == '"' and v[-1] == '"':
            v = v[1:-1]
        if v == "":
            out[k] = ""
            continue
        try:
            if "." in v or "e" in v or "E" in v:
                out[k] = float(v)
            else:
                out[k] = int(v)
        except ValueError:
            out[k] = v
    return out

def _wait_for_metrics_stable(path: str, timeout_s: float) -> bool:
    t0 = time.time()
    last_size = None
    stable_hits = 0
    while time.time() - t0 < timeout_s:
        if os.path.exists(path):
            try:
                size = os.path.getsize(path)
            except OSError:
                size = None
            if size is not None:
                if last_size is not None and size == last_size and size > 0:
                    stable_hits += 1
                else:
                    stable_hits = 0
                last_size = size
                if stable_hits >= 2:
                    return True
        time.sleep(0.2)
    return False

def _initial_g0_bounds_stds() -> Tuple[List[float], List[float], List[float], List[float]]:
    """Build initial CMA mean (g0), per-dim bounds in g-space, and per-dim initial stds.

    g-space:
      - linear params: g = x
      - log params:    g = ln(x)
    """
    g0: List[float] = []
    lb: List[float] = []
    ub: List[float] = []
    stds: List[float] = []

    for k in PARAM_KEYS:
        lo, hi = PARAM_BOUNDS[k]
        x0 = float(PARAM_INIT.get(k, (lo + hi) * 0.5))
        # clamp init to bounds
        x0 = max(lo, min(hi, x0))

        if k in LOG_PARAMS:
            if lo <= 0.0 or hi <= 0.0:
                raise ValueError(f"LOG param must have positive bounds: {k} bounds=({lo},{hi})")
            g0.append(math.log(x0))
            lb.append(math.log(lo))
            ub.append(math.log(hi))
            stds.append(LOG_SIGMA_FRAC * math.log(hi / lo))
        else:
            g0.append(x0)
            lb.append(lo)
            ub.append(hi)
            stds.append(LIN_SIGMA_FRAC * (hi - lo))

    return g0, lb, ub, stds

def theta_x_from_g_vector(g_vec: List[float]) -> Dict[str, float]:
    """Convert CMA genotype vector g -> parameter dict in original units (x-space)."""
    if len(g_vec) != len(PARAM_KEYS):
        raise ValueError(f"Bad vector size: {len(g_vec)} vs {len(PARAM_KEYS)}")

    th: Dict[str, float] = {}
    for i, k in enumerate(PARAM_KEYS):
        lo, hi = PARAM_BOUNDS[k]
        g = float(g_vec[i])

        if k in LOG_PARAMS:
            x = math.exp(g)
        else:
            x = g

        # safety clamp (should be redundant when CMA bounds are configured correctly)
        if x < lo:
            x = lo
        elif x > hi:
            x = hi

        th[k] = float(x)
    return th

def compute_cost(metrics: Dict[str, Any]) -> float:
    crashed = int(metrics.get("crashed", 0)) == 1
    if crashed:
        crash_time = float(metrics.get("crash_time_s", -1.0))
        return CRASH_PENALTY + CRASH_TIME_PENALTY * max(0.0, DEFAULT_SIM_TIME_S - max(0.0, crash_time))

    J = 0.0
    for k, w in COST_WEIGHTS.items():
        val = metrics.get(k, 0.0)
        try:
            valf = float(val)
        except Exception:
            valf = 0.0
        J += w * valf
    return J

def _append_checkpoint(episodes_done: int, best_cost: float, best_theta: Dict[str, float]) -> None:
    rec = {
        "episodes_done": int(episodes_done),
        "best_cost": float(best_cost),
        "best_theta": {k: float(v) for k, v in best_theta.items()},
        "timestamp_unix": float(time.time()),
    }
    os.makedirs(os.path.dirname(CHECKPOINT_LOG_PATH), exist_ok=True)
    with open(CHECKPOINT_LOG_PATH, "a") as f:
        f.write(json.dumps(rec) + "\n")

@dataclass
class EpisodeResult:
    cost: float
    metrics: Dict[str, Any]
    track_index: int
    crashed: bool

def _build_sim_launch_args(track_index: int, sim_time_s: int) -> Dict[str, str]:
    return {
        "sim_time": str(sim_time_s),
        "track_id": str(track_index),
        "low_level_controlers": str(SIM_LOW_LEVEL_CONTROLERS),
        "ins_mode": str(SIM_INS_MODE),
    }

def _popen_roslaunch(
    launch_file: str,
    launch_args: Dict[str, str],
    extra_env: Optional[Dict[str, str]] = None,
) -> subprocess.Popen:
    cmd = ["roslaunch", launch_file] + [f"{k}:={v}" for k, v in launch_args.items()]
    env = _ros_env(extra_env)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        text=True,
        bufsize=1,
        universal_newlines=True,
    )
    p._cmd = cmd
    return p

def _kill_process_group(p: Optional[subprocess.Popen], sig=signal.SIGINT, timeout_s: float = 5.0) -> None:
    if p is None:
        return
    try:
        pgid = os.getpgid(p.pid)
    except Exception:
        pgid = None
    try:
        if pgid is not None:
            os.killpg(pgid, sig)
        else:
            p.send_signal(sig)
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if p.poll() is not None:
            break
        time.sleep(0.1)
    try:
        if pgid is not None:
            os.killpg(pgid, signal.SIGKILL)
        else:
            p.kill()
    except Exception:
        pass

def apply_params_to_control_json(theta_x: Dict[str, float]) -> None:
    data = _load_json(CONTROL_PARAM_JSON)
    for path, val in theta_x.items():
        if path not in PARAM_BOUNDS:
            continue
        _set_json_path(data, path, float(val))
    _save_json_atomic(CONTROL_PARAM_JSON, data)

def run_one_episode(theta_x: Dict[str, float], track_index: int, sim_time_s: int = DEFAULT_SIM_TIME_S) -> EpisodeResult:
    global EPISODES_DONE, NEXT_LOG_AT, BEST_COST_SO_FAR, BEST_THETA_SO_FAR

    apply_params_to_control_json(theta_x)

    try:
        if os.path.exists(METRICS_CSV):
            os.remove(METRICS_CSV)
    except Exception:
        pass

    sim_args = _build_sim_launch_args(track_index, sim_time_s)
    sim_p = _popen_roslaunch(SIM_LAUNCH, sim_args)

    ctrl_env = {"LD_LIBRARY_PATH": f"{ACADOS_LIB}:" + os.environ.get("LD_LIBRARY_PATH", "")}
    ctrl_p = _popen_roslaunch(CTRL_LAUNCH, {}, extra_env=ctrl_env)

    time.sleep(1.0)

    if sim_p.poll() is not None:
        rc = int(sim_p.returncode)
        reason = f"sim_launch_failed_rc={rc}"
        metrics = {"crashed": 1, "crash_reason": reason, "crash_time_s": -1}
        cost = compute_cost(metrics)
        print(f"[EP] track={track_index} crashed=1 reason={reason} cost={cost:.6g}")
        _kill_process_group(sim_p)
        _kill_process_group(ctrl_p)
        res = EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=True)

    elif ctrl_p.poll() is not None:
        rc = int(ctrl_p.returncode)
        reason = f"ctrl_launch_failed_rc={rc}"
        metrics = {"crashed": 1, "crash_reason": reason, "crash_time_s": -1}
        cost = compute_cost(metrics)
        print(f"[EP] track={track_index} crashed=1 reason={reason} cost={cost:.6g}")
        _kill_process_group(sim_p)
        _kill_process_group(ctrl_p)
        res = EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=True)

    else:
        t0 = time.time()
        while True:
            if sim_p.poll() is not None:
                break
            if time.time() - t0 > sim_time_s + EPISODE_HARD_TIMEOUT_MARGIN_S:
                break
            time.sleep(0.2)

        _kill_process_group(sim_p)
        _kill_process_group(ctrl_p)

        ok = _wait_for_metrics_stable(METRICS_CSV, METRICS_WAIT_TIMEOUT_S)
        if not ok:
            metrics = {"crashed": 1, "crash_reason": "no_metrics_file", "crash_time_s": -1}
            cost = compute_cost(metrics)
            print(f"[EP] track={track_index} crashed=1 reason=no_metrics_file cost={cost:.6g}")
            res = EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=True)
        else:
            metrics = _parse_metrics_csv(METRICS_CSV)
            cost = compute_cost(metrics)
            crashed = int(metrics.get("crashed", 0)) == 1
            reason = metrics.get("crash_reason", "")
            print(f"[EP] track={track_index} crashed={1 if crashed else 0} reason={reason} cost={cost:.6g}")
            res = EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=crashed)

    EPISODES_DONE += 1
    while EPISODES_DONE >= NEXT_LOG_AT:
        try:
            _append_checkpoint(NEXT_LOG_AT, BEST_COST_SO_FAR, BEST_THETA_SO_FAR)
            print(f"[CHK] saved @episodes={NEXT_LOG_AT} -> {CHECKPOINT_LOG_PATH}")
        except Exception as e:
            print(f"[CHK] WARN cannot write checkpoint: {e}", file=sys.stderr)
        NEXT_LOG_AT += LOG_EVERY_EPISODES

    return res


# ===== deterministic track schedule =====
def sample_k_tracks_with_replacement_rng(rng: random.Random, k: int) -> List[int]:
    # "zastępstwo" (with replacement) => zawsze k sztuk, proste i deterministyczne
    return [rng.randint(TRACK_INDEX_MIN, TRACK_INDEX_MAX) for _ in range(k)]


# ===== deterministic track schedule per generation (for fair comparisons) =====
TRACKS_SEED = TRAIN_SEED

def tracks_for_generation(generation_id: int, k: int) -> List[int]:
    total = TRACK_INDEX_MAX - TRACK_INDEX_MIN + 1
    kk = max(1, min(int(k), int(total)))
    rng = random.Random(int(TRACKS_SEED) + 999_983 + int(generation_id) * 1_000_003)
    ids = list(range(TRACK_INDEX_MIN, TRACK_INDEX_MAX + 1))
    rng.shuffle(ids)
    return ids[:kk]
def evaluate_theta_3runs(theta_x: Dict[str, float], tracks: List[int]) -> Tuple[float, Dict[str, Any]]:
    tracks = list(tracks)
    costs = []
    crashes = 0
    for tid in tracks:
        res = run_one_episode(theta_x, tid, DEFAULT_SIM_TIME_S)
        costs.append(res.cost)
        if res.crashed:
            crashes += 1
    mean_cost = sum(costs) / max(1, len(costs))
    info = {"tracks": tracks, "crashes": crashes, "episodes": len(tracks), "mean_cost": mean_cost}
    return mean_cost, info

def evaluate_population_3runs(Y: List[List[float]], tracks: List[int]) -> Tuple[List[float], Dict[str, Any]]:
    F: List[float] = []
    dbg = {"tracks_per_candidate": []}
    for yi in Y:
        theta = theta_x_from_g_vector(yi)
        J, info = evaluate_theta_3runs(theta, tracks)
        F.append(J)
        dbg["tracks_per_candidate"].append(info["tracks"])
    return F, dbg

def main():
    global BEST_COST_SO_FAR, BEST_THETA_SO_FAR, EPISODES_DONE, NEXT_LOG_AT

    try:
        import cma
    except ImportError:
        print("Brak biblioteki 'cma'. Zainstaluj: pip install cma", file=sys.stderr)
        sys.exit(1)

    # ===== SEED (tylko to) =====
    random.seed(TRAIN_SEED)
    try:
        import numpy as np
        np.random.seed(TRAIN_SEED)
    except Exception:
        pass

    # osobny deterministyczny RNG do torów (NIE DOTYKAĆ nigdzie indziej)
    rng_tracks = random.Random(TRAIN_SEED + 999)  # unused (tracks are generation-indexed)

    # reset liczników logowania
    EPISODES_DONE = 0
    NEXT_LOG_AT = LOG_EVERY_EPISODES
    BEST_COST_SO_FAR = float("inf")
    BEST_THETA_SO_FAR = {}

    # checkpoint startowy (0)
    try:
        _append_checkpoint(0, BEST_COST_SO_FAR, BEST_THETA_SO_FAR)
    except Exception:
        pass

    try:
        _must_exist(SIM_LAUNCH, "SIM_LAUNCH")
        _must_exist(CTRL_LAUNCH, "CTRL_LAUNCH")
        _must_exist(CONTROL_PARAM_JSON, "CONTROL_PARAM_JSON")
    except Exception as e:
        print("[FATAL]", e, file=sys.stderr)
        sys.exit(2)

    roscore_p = None
    if START_ROSCORE_IF_NEEDED:
        if not _rosmaster_is_up():
            roscore_p = _popen_roscore()
            time.sleep(1.0)
            if not _rosmaster_is_up():
                print("[FATAL] roscore did not come up.", file=sys.stderr)
                _kill_process_group(roscore_p)
                sys.exit(3)
            print(f"[INFO] Started roscore on {ROS_MASTER_HOST}:{ROS_MASTER_PORT} (EXPERIMENT_ID={EXPERIMENT_ID}).")
        else:
            print(f"[INFO] rosmaster already running on {ROS_MASTER_HOST}:{ROS_MASTER_PORT}, reusing it.")

    # ===== CMA init in g-space + bounds =====
    # g0: initial mean in CMA genotype space
    # lb/ub: per-dim bounds in genotype space (log params already ln-bounded)
    # stds: per-dim initial 1σ in genotype space (see LOG_SIGMA_FRAC / LIN_SIGMA_FRAC)
    try:
        g0, lb, ub, stds = _initial_g0_bounds_stds()
    except Exception as e:
        print("[FATAL] invalid bounds/init for CMA:", e, file=sys.stderr)
        sys.exit(4)

    theta_x0 = theta_x_from_g_vector(g0)

    # ===== CMA seed + bounds handling =====
    # NOTE: sigma0 is set to 1.0 and the actual per-dim stds are passed via CMA_stds.
    opts = {
        "popsize": CMA_LAMBDA,
        "maxiter": CMA_MAXGEN,
        "seed": TRAIN_SEED,
        "bounds": [lb, ub],
        "CMA_stds": stds,
    }
    es = cma.CMAEvolutionStrategy(g0, 1.0, opts)

    best_cost = float("inf")
    best_theta_x = theta_x0.copy()
    BEST_COST_SO_FAR = best_cost
    BEST_THETA_SO_FAR = best_theta_x.copy()

    print("\n=== CMA-ES (3 runs per candidate, deterministic tracks) ===")
    print(f"[SEED] TRAIN_SEED={TRAIN_SEED} | tracks_seed={TRAIN_SEED+999}")
    print(f"[BUDGET] TOTAL={TOTAL_EPISODES_BUDGET} | per_gen={EPISODES_PER_GEN} | CMA_MAXGEN={CMA_MAXGEN} -> {CMA_MAXGEN*EPISODES_PER_GEN} + extra {EXTRA_EPISODES}")
    print(f"[CHECKPOINT] every {LOG_EVERY_EPISODES} episodes -> {CHECKPOINT_LOG_PATH}\n")

    gen = 0
    try:
        while (not es.stop()) and gen < CMA_MAXGEN:
            gen += 1
            gen_t0 = time.time()

            print(f"\n=== GENERATION {gen}/{CMA_MAXGEN} ===")

            Y = es.ask()
            tracks_gen = tracks_for_generation(gen, EPISODES_PER_CANDIDATE)
            F, dbg = evaluate_population_3runs(Y, tracks_gen)

            for yi, Ji in zip(Y, F):
                if Ji < best_cost:
                    best_cost = Ji
                    best_theta_x = theta_x_from_g_vector(yi)

            if best_cost < BEST_COST_SO_FAR:
                BEST_COST_SO_FAR = float(best_cost)
                BEST_THETA_SO_FAR = {k: float(v) for k, v in best_theta_x.items()}

            # opcjonalnie: debug jakie tracki poszły
            # print(f"[GEN] tracks_per_candidate={dbg['tracks_per_candidate']}")

            es.tell(Y, F)
            es.disp()

            best_out_path = os.path.join(os.path.dirname(CONTROL_PARAM_JSON), "control_param_best_from_cma_3runs.json")
            try:
                data = _load_json(CONTROL_PARAM_JSON)
                for path, val in best_theta_x.items():
                    _set_json_path(data, path, float(val))
                _save_json_atomic(best_out_path, data)
                print(f"[CMA] Saved best params -> {best_out_path}")
            except Exception as e:
                print(f"[CMA] WARN: cannot save best json: {e}", file=sys.stderr)

            gen_time_s = time.time() - gen_t0
            print(f"[GEN] time={gen_time_s:.2f}s | episodes_done={EPISODES_DONE}/{TOTAL_EPISODES_BUDGET} | best_cost={best_cost:.6g}")

        # ===== dobijamy do 1024 epizodów (extra eval best-so-far, nie wpływa na CMA) =====
        if EXTRA_EPISODES > 0 and EPISODES_DONE < TOTAL_EPISODES_BUDGET:
            print(f"\n[EXTRA] Running {EXTRA_EPISODES} extra episodes with best-so-far to hit budget...")
            extra_tracks = tracks_for_generation(CMA_MAXGEN + 1, EXTRA_EPISODES)
            for tid in extra_tracks:
                _ = run_one_episode(best_theta_x, tid, DEFAULT_SIM_TIME_S)
            print(f"[EXTRA] done. episodes_done={EPISODES_DONE}/{TOTAL_EPISODES_BUDGET}")

    finally:
        if roscore_p is not None:
            _kill_process_group(roscore_p)

    # final checkpoint
    try:
        _append_checkpoint(EPISODES_DONE, BEST_COST_SO_FAR, BEST_THETA_SO_FAR)
    except Exception:
        pass

    print("\n=== DONE ===")
    print(f"Episodes done: {EPISODES_DONE}/{TOTAL_EPISODES_BUDGET}")
    print("Best theta (x-space):")
    for k, v in best_theta_x.items():
        lo, hi = PARAM_BOUNDS[k]
        print(f"  {k}: {v:.6g}  (bounds [{lo}, {hi}])")
    print(f"Checkpoints saved at: {CHECKPOINT_LOG_PATH}")

if __name__ == "__main__":
    main()
