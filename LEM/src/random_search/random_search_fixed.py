#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import math
import signal
import subprocess
import socket
import random
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple, List

# =========================
# USER CONFIG (EDYTUJ TUTAJ)
# =========================

EXPERIMENT_ID = "fs0"
WORKSPACE_ROOT = os.path.expanduser("~/fs_control_simulator/LEM/src")

SIM_LAUNCH  = os.path.join(WORKSPACE_ROOT, "lem_simulator", "launch", "sim.launch")
CTRL_LAUNCH = os.path.join(WORKSPACE_ROOT, "dv_control", "launch", "control.launch")

CONTROL_PARAM_JSON = os.path.join(WORKSPACE_ROOT, "dv_control", "config", "Params", "control_param.json")
METRICS_CSV         = os.path.join(WORKSPACE_ROOT, "lem_simulator", "logs", "run_default_metrics.csv")

DEFAULT_SIM_TIME_S = 120
ACADOS_LIB = os.path.expanduser("~/fs_control_simulator/LEM/src/dv_control/External/acados/install/lib")

TRACK_INDEX_MIN = 1
TRACK_INDEX_MAX = 50

SIM_INS_MODE = "kalman"
SIM_LOW_LEVEL_CONTROLERS = "false"

# ===== koszt (jak u Ciebie) =====
COST_WEIGHTS = {
    "vs_avg_mps": -2.0,
    "ey_avg_m": 30.0,
    "kappa_metric": 175.0,
    "slip_angle_metric": 175.0,
}
CRASH_PENALTY = 15.0
CRASH_TIME_PENALTY = 0.05

# ===== przestrzeń parametrów =====
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

# ===== log-scale tylko dla wag MPC ("cost") =====
LOG_PARAMS = {
    "mpc.cost.Q_y",
    "mpc.cost.Q_psi",
    "mpc.cost.R_ddelta",
}

# ===== budżet =====
TOTAL_EPISODES_BUDGET = 1024
EPISODES_PER_TRIAL = 3

# ile triali i ile „reszty” żeby dobić idealnie do 1024
TRIALS_TOTAL = TOTAL_EPISODES_BUDGET // EPISODES_PER_TRIAL          # 341 -> 1023 epizody
REMAINING_EPISODES = TOTAL_EPISODES_BUDGET - TRIALS_TOTAL * EPISODES_PER_TRIAL  # 1 epizod

# ===== schemat generacji (LHS startup -> dogęszczanie) =====
STARTUP_TRIALS = 60          # LHS na start
GEN_TRIALS = 40              # ile triali w jednej generacji (łącznie)
ELITE_SIZE = 10              # top-k utrzymujemy
RANDOM_FRACTION = 0.10       # ~10% prób w każdej generacji to czysty random (anti-stuck)

# sigma w przestrzeni znormalizowanej [0,1]^d (po log/lin transform)
SIGMA0 = 0.25
SIGMA_DECAY = 0.70
SIGMA_MIN = 0.03

# ===== clipy metryk (heurystyczne) =====
METRIC_CLIPS = {
    "vs_avg_mps": (0.0, 40.0),
    "ey_avg_m": (0.0, 3.0),
    "kappa_metric": (0.0, 5.0),
    "slip_angle_metric": (0.0, 1.2),
}

# ===== logi =====
RUN_LOG_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__) or "."), "gensearch_run_log.jsonl")
BEST_JSON_OUT = os.path.join(os.path.dirname(CONTROL_PARAM_JSON), "control_param_best_from_gensearch.json")

CHECKPOINT_LOG_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__) or "."), "gensearch_checkpoints.jsonl")
CHECKPOINT_EVERY = 50

METRICS_WAIT_TIMEOUT_S = 10.0
EPISODE_HARD_TIMEOUT_MARGIN_S = 30.0

# ===== ROS master =====
ROS_MASTER_HOST = "127.0.0.1"
ROS_MASTER_PORT = 11311
START_ROSCORE_IF_NEEDED = True

# ===== ROS isolation =====
ROS_HOME = os.path.expanduser(f"~/.ros_{EXPERIMENT_ID}")
ROS_LOG_DIR = os.path.join(ROS_HOME, "log")

# ===== RNG =====
GLOBAL_SEED = 123
os.environ.setdefault("PYTHONHASHSEED", str(GLOBAL_SEED))


# =========================
#  CHECKPOINT LOG
# =========================

def _append_checkpoint(episodes_done: int, best_cost: float, best_theta_x: Dict[str, float]) -> None:
    rec = {
        "episodes_done": int(episodes_done),
        "best_cost": float(best_cost),
        "best_theta": {k: float(v) for k, v in best_theta_x.items()},
        "timestamp_unix": float(time.time()),
    }
    os.makedirs(os.path.dirname(CHECKPOINT_LOG_PATH), exist_ok=True)
    with open(CHECKPOINT_LOG_PATH, "a") as f:
        f.write(json.dumps(rec) + "\n")


def _maybe_checkpoint(episodes_done: int, best_cost: float, best_theta: Dict[str, float], last_logged_bucket: int) -> int:
    bucket = episodes_done // CHECKPOINT_EVERY
    if bucket > last_logged_bucket:
        snap = bucket * CHECKPOINT_EVERY
        _append_checkpoint(snap, best_cost, best_theta)
        return bucket
    return last_logged_bucket


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
        if not line.strip():
            continue
        if "," not in line:
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


def _clip_metric(name: str, x: float) -> float:
    if name in METRIC_CLIPS:
        lo, hi = METRIC_CLIPS[name]
        if x < lo:
            return lo
        if x > hi:
            return hi
    return x


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
        valf = _clip_metric(k, valf)
        J += w * valf
    return J


@dataclass
class EpisodeResult:
    cost: float
    metrics: Dict[str, Any]
    track_index: int
    crashed: bool


def sample_k_unique_tracks_rng(rng: random.Random, k: int) -> List[int]:
    total = TRACK_INDEX_MAX - TRACK_INDEX_MIN + 1
    k = max(1, min(k, total))
    ids = list(range(TRACK_INDEX_MIN, TRACK_INDEX_MAX + 1))
    rng.shuffle(ids)
    return ids[:k]

# ===== deterministic track schedule per generation =====
# For fair comparisons:
# - Within a generation: ALL candidates use the SAME 3 tracks.
# - Across methods: generation_id -> identical track set (if seeds / bounds match).
TRACKS_SEED = GLOBAL_SEED

def tracks_for_generation(generation_id: int, k: int) -> List[int]:
    total = TRACK_INDEX_MAX - TRACK_INDEX_MIN + 1
    kk = max(1, min(int(k), int(total)))
    # deterministic, generation-indexed RNG (same formula used in other scripts)
    rng = random.Random(int(TRACKS_SEED) + 999_983 + int(generation_id) * 1_000_003)
    ids = list(range(TRACK_INDEX_MIN, TRACK_INDEX_MAX + 1))
    rng.shuffle(ids)
    return ids[:kk]


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


def _clamp_param(k: str, x: float) -> float:
    lo, hi = PARAM_BOUNDS[k]
    if lo > hi:
        lo, hi = hi, lo
    xf = float(x)
    if xf < lo:
        return float(lo)
    if xf > hi:
        return float(hi)
    return xf


def apply_params_to_control_json(theta_x: Dict[str, float]) -> None:
    data = _load_json(CONTROL_PARAM_JSON)
    for path, val in theta_x.items():
        if path not in PARAM_BOUNDS:
            continue
        _set_json_path(data, path, _clamp_param(path, val))
    _save_json_atomic(CONTROL_PARAM_JSON, data)


def run_one_episode(theta_x: Dict[str, float], track_index: int, sim_time_s: int = DEFAULT_SIM_TIME_S) -> EpisodeResult:
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
        return EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=True)

    if ctrl_p.poll() is not None:
        rc = int(ctrl_p.returncode)
        reason = f"ctrl_launch_failed_rc={rc}"
        metrics = {"crashed": 1, "crash_reason": reason, "crash_time_s": -1}
        cost = compute_cost(metrics)
        print(f"[EP] track={track_index} crashed=1 reason={reason} cost={cost:.6g}")
        _kill_process_group(sim_p)
        _kill_process_group(ctrl_p)
        return EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=True)

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
        return EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=True)

    metrics = _parse_metrics_csv(METRICS_CSV)
    cost = compute_cost(metrics)
    crashed = int(metrics.get("crashed", 0)) == 1
    reason = metrics.get("crash_reason", "")
    print(f"[EP] track={track_index} crashed={1 if crashed else 0} reason={reason} cost={cost:.6g}")
    return EpisodeResult(cost=cost, metrics=metrics, track_index=track_index, crashed=crashed)


def evaluate_theta_on_tracks(theta_x: Dict[str, float], track_ids: List[int], sim_time_s: int = DEFAULT_SIM_TIME_S) -> Tuple[float, Dict[str, Any], int]:
    costs = []
    crashes = 0
    spent = 0
    for tid in track_ids:
        res = run_one_episode(theta_x, tid, sim_time_s)
        spent += 1
        costs.append(res.cost)
        if res.crashed:
            crashes += 1
    mean_cost = sum(costs) / max(1, len(costs))
    info = {"tracks": track_ids, "crashes": crashes, "episodes": len(track_ids), "mean_cost": mean_cost}
    return mean_cost, info, spent


def _append_log(rec: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(RUN_LOG_PATH), exist_ok=True)
    r = dict(rec)
    r["timestamp_unix"] = float(time.time())
    with open(RUN_LOG_PATH, "a") as f:
        f.write(json.dumps(r) + "\n")


def _save_best_json(theta_best: Dict[str, float]) -> None:
    data = _load_json(CONTROL_PARAM_JSON)
    for path, val in theta_best.items():
        _set_json_path(data, path, _clamp_param(path, val))
    _save_json_atomic(BEST_JSON_OUT, data)


# =========================
#  Sampling (LHS startup + global + local around elites)
# =========================

PARAM_KEYS = list(PARAM_BOUNDS.keys())

def _param_to_u(k: str, x: float) -> float:
    lo, hi = PARAM_BOUNDS[k]
    if k in LOG_PARAMS:
        t = math.log(max(float(x), 1e-300))
        tlo = math.log(lo)
        thi = math.log(hi)
    else:
        t = float(x)
        tlo = float(lo)
        thi = float(hi)

    if thi == tlo:
        return 0.5
    return (t - tlo) / (thi - tlo)


def _u_to_param(k: str, u: float) -> float:
    lo, hi = PARAM_BOUNDS[k]
    uu = float(u)
    if uu < 0.0:
        uu = 0.0
    if uu > 1.0:
        uu = 1.0

    if k in LOG_PARAMS:
        tlo = math.log(lo)
        thi = math.log(hi)
        t = tlo + uu * (thi - tlo)
        x = math.exp(t)
    else:
        tlo = float(lo)
        thi = float(hi)
        x = tlo + uu * (thi - tlo)

    return _clamp_param(k, x)


def generate_lhs_startup_thetas(rng: random.Random, n_samples: int) -> List[Dict[str, float]]:
    n = int(n_samples)
    if n <= 0:
        return []

    d = len(PARAM_KEYS)

    strata = [list(range(n)) for _ in range(d)]
    for j in range(d):
        rng.shuffle(strata[j])

    thetas: List[Dict[str, float]] = []
    for i in range(n):
        th: Dict[str, float] = {}
        for j, k in enumerate(PARAM_KEYS):
            u = (strata[j][i] + rng.random()) / float(n)  # u in [0,1)
            th[k] = float(_u_to_param(k, u))
        thetas.append(th)

    return thetas


def _reflect01(u: float) -> float:
    x = float(u)
    for _ in range(8):
        if 0.0 <= x <= 1.0:
            return x
        if x < 0.0:
            x = -x
        if x > 1.0:
            x = 2.0 - x
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


def sample_theta_global(rng: random.Random) -> Dict[str, float]:
    th: Dict[str, float] = {}
    for k, (lo, hi) in PARAM_BOUNDS.items():
        if k in LOG_PARAMS:
            u = rng.random()
            x = math.exp(math.log(lo) + (math.log(hi) - math.log(lo)) * u)
        else:
            x = float(lo) + (float(hi) - float(lo)) * rng.random()
        th[k] = float(_clamp_param(k, x))
    return th


def _pick_elite_index(rng: random.Random, elites: List[Tuple[float, Dict[str, float]]]) -> int:
    if not elites:
        return 0
    weights = []
    for i in range(len(elites)):
        weights.append(1.0 / float(i + 1))
    s = sum(weights)
    r = rng.random() * s
    acc = 0.0
    for i, w in enumerate(weights):
        acc += w
        if acc >= r:
            return i
    return len(elites) - 1


def sample_theta_local(rng: random.Random, elites: List[Tuple[float, Dict[str, float]]], sigma: float) -> Dict[str, float]:
    if not elites:
        return sample_theta_global(rng)

    base_idx = _pick_elite_index(rng, elites)
    base = elites[base_idx][1]

    th: Dict[str, float] = {}
    for k in PARAM_BOUNDS.keys():
        u0 = _param_to_u(k, base[k])
        u = u0 + float(sigma) * rng.gauss(0.0, 1.0)
        u = _reflect01(u)
        th[k] = float(_u_to_param(k, u))
    return th


def _select_elites(results: List[Tuple[float, Dict[str, float]]], k: int) -> List[Tuple[float, Dict[str, float]]]:
    if not results:
        return []
    rs = sorted(results, key=lambda t: float(t[0]))
    return rs[: max(1, min(k, len(rs)))]


# =========================
#  MAIN: Generational search (LHS startup)
# =========================

def main() -> None:
    try:
        _must_exist(SIM_LAUNCH, "SIM_LAUNCH")
        _must_exist(CTRL_LAUNCH, "CTRL_LAUNCH")
        _must_exist(CONTROL_PARAM_JSON, "CONTROL_PARAM_JSON")
    except Exception as e:
        print("[FATAL]", e, file=sys.stderr)
        sys.exit(2)

    if STARTUP_TRIALS >= TRIALS_TOTAL:
        print("[FATAL] STARTUP_TRIALS must be < TRIALS_TOTAL", file=sys.stderr)
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

    # deterministyczne RNG: osobno tory i osobno parametry
    rng_tracks = random.Random(GLOBAL_SEED + 999)
    rng_theta  = random.Random(GLOBAL_SEED + 12345)
    rng_lhs    = random.Random(GLOBAL_SEED + 77777)

    startup_thetas = generate_lhs_startup_thetas(rng_lhs, STARTUP_TRIALS)

    episodes_done = 0
    trial_idx = 0

    best_cost = float("inf")
    best_theta: Dict[str, float] = {}
    for k, (lo, hi) in PARAM_BOUNDS.items():
        x = float(PARAM_INIT.get(k, (float(lo) + float(hi)) * 0.5))
        best_theta[k] = float(_clamp_param(k, x))

    last_logged_bucket = -1
    try:
        _append_checkpoint(0, best_cost, best_theta)
        last_logged_bucket = 0
    except Exception:
        pass

    after_start = TRIALS_TOTAL - STARTUP_TRIALS
    n_full_gens = after_start // GEN_TRIALS
    tail_trials = after_start - n_full_gens * GEN_TRIALS

    print("\n=== GENERATIONAL SEARCH (LHS STARTUP) ===")
    print(f"[SEED] GLOBAL_SEED={GLOBAL_SEED}")
    print(f"[BUDGET] TOTAL_EPISODES_BUDGET={TOTAL_EPISODES_BUDGET} => trials={TRIALS_TOTAL} (each {EPISODES_PER_TRIAL} eps) + remaining_eps={REMAINING_EPISODES}")
    print(f"[PLAN] startup_trials(LHS)={STARTUP_TRIALS} | full_gens={n_full_gens} x gen_trials={GEN_TRIALS} | tail_trials={tail_trials}")
    print(f"[GEN] elite={ELITE_SIZE} | random_frac={RANDOM_FRACTION:.2f} | sigma0={SIGMA0} decay={SIGMA_DECAY} min={SIGMA_MIN}")
    print(f"[LOG] run log    : {RUN_LOG_PATH}")
    print(f"[LOG] best json  : {BEST_JSON_OUT}")
    print(f"[LOG] checkpoints: {CHECKPOINT_LOG_PATH} (every {CHECKPOINT_EVERY} episodes)\n")

    results_pool: List[Tuple[float, Dict[str, float]]] = []
    elites: List[Tuple[float, Dict[str, float]]] = []

    try:
        # --- startup: LHS ---
        for i in range(STARTUP_TRIALS):
            if trial_idx >= TRIALS_TOTAL:
                break
            trial_idx += 1

            theta = startup_thetas[i]
            tracks = tracks_for_generation(0, EPISODES_PER_TRIAL)

            J, info, spent = evaluate_theta_on_tracks(theta, tracks)
            episodes_done += spent
            last_logged_bucket = _maybe_checkpoint(episodes_done, best_cost, best_theta, last_logged_bucket)

            improved = False
            if J < best_cost:
                best_cost = float(J)
                best_theta = {k: float(v) for k, v in theta.items()}
                improved = True
                try:
                    _save_best_json(best_theta)
                except Exception:
                    pass

            results_pool.append((float(J), {k: float(v) for k, v in theta.items()}))
            _append_log({
                "phase": "startup_lhs",
                "gen": 0,
                "sigma": None,
                "trial": int(trial_idx),
                "tracks": tracks,
                "mean_cost": float(J),
                "episodes_done": int(episodes_done),
                "crashes": int(info.get("crashes", 0)),
                "theta": {k: float(v) for k, v in theta.items()},
                "improved": bool(improved),
            })

            print(f"[TRIAL {trial_idx:04d}] phase=startup_lhs tracks={tracks} mean_cost={J:.6g} crashes={info.get('crashes',0)}/{info.get('episodes',0)}"
                  f"{'  **BEST**' if improved else ''} | episodes={episodes_done}/{TOTAL_EPISODES_BUDGET}")

        elites = _select_elites(results_pool, ELITE_SIZE)

        # --- generacje: re-eval elit + lokalne dogęszczanie + ~10% random ---
        for gen in range(1, n_full_gens + 1 + (1 if tail_trials > 0 else 0)):
            if trial_idx >= TRIALS_TOTAL:
                break

            gen_trials = GEN_TRIALS
            if gen == n_full_gens + 1 and tail_trials > 0:
                gen_trials = tail_trials

            sigma = max(SIGMA_MIN, SIGMA0 * (SIGMA_DECAY ** float(gen - 1)))

            n_rand = int(math.ceil(gen_trials * RANDOM_FRACTION))
            if n_rand < 1:
                n_rand = 1

            n_elite_eval = min(len(elites), min(ELITE_SIZE, gen_trials))
            n_local = gen_trials - n_elite_eval - n_rand
            if n_local < 0:
                n_local = 0
                n_rand = max(0, gen_trials - n_elite_eval)

            candidates: List[Tuple[str, Dict[str, float]]] = []

            for i in range(n_elite_eval):
                candidates.append(("elite", {k: float(v) for k, v in elites[i][1].items()}))

            for _ in range(n_local):
                candidates.append(("local", sample_theta_local(rng_theta, elites, sigma)))

            for _ in range(n_rand):
                candidates.append(("random", sample_theta_global(rng_theta)))

            results_gen: List[Tuple[float, Dict[str, float]]] = []

            tracks_gen = tracks_for_generation(gen, EPISODES_PER_TRIAL)

            print(f"\n[GEN {gen}] trials={gen_trials} -> elite_eval={n_elite_eval}, local={n_local}, random={n_rand} | sigma={sigma:.4g}")

            for kind, theta in candidates:
                if trial_idx >= TRIALS_TOTAL:
                    break
                trial_idx += 1

                tracks = tracks_gen
                J, info, spent = evaluate_theta_on_tracks(theta, tracks)
                episodes_done += spent
                last_logged_bucket = _maybe_checkpoint(episodes_done, best_cost, best_theta, last_logged_bucket)

                improved = False
                if J < best_cost:
                    best_cost = float(J)
                    best_theta = {k: float(v) for k, v in theta.items()}
                    improved = True
                    try:
                        _save_best_json(best_theta)
                    except Exception:
                        pass

                results_gen.append((float(J), {k: float(v) for k, v in theta.items()}))

                _append_log({
                    "phase": "gen",
                    "gen": int(gen),
                    "sigma": float(sigma),
                    "kind": str(kind),
                    "trial": int(trial_idx),
                    "tracks": tracks,
                    "mean_cost": float(J),
                    "episodes_done": int(episodes_done),
                    "crashes": int(info.get("crashes", 0)),
                    "theta": {k: float(v) for k, v in theta.items()},
                    "improved": bool(improved),
                })

                print(f"[TRIAL {trial_idx:04d}] gen={gen} kind={kind:<6} tracks={tracks} mean_cost={J:.6g} crashes={info.get('crashes',0)}/{info.get('episodes',0)}"
                      f"{'  **BEST**' if improved else ''} | episodes={episodes_done}/{TOTAL_EPISODES_BUDGET}")

            elites = _select_elites(results_gen, ELITE_SIZE)

        # --- dobijamy do 1024 epizodów (1 ep) na best ---
        if REMAINING_EPISODES > 0 and episodes_done < TOTAL_EPISODES_BUDGET:
            extra = min(REMAINING_EPISODES, TOTAL_EPISODES_BUDGET - episodes_done)
            extra_tracks = tracks_for_generation((n_full_gens + (1 if tail_trials > 0 else 0)) + 1, extra)

            print(f"\n[EXTRA] running {extra} episode(s) on tracks={extra_tracks} using BEST theta-so-far")
            Jx, infox, spentx = evaluate_theta_on_tracks(best_theta, extra_tracks)
            episodes_done += spentx
            last_logged_bucket = _maybe_checkpoint(episodes_done, best_cost, best_theta, last_logged_bucket)

            _append_log({
                "phase": "extra",
                "gen": None,
                "sigma": None,
                "trial": "EXTRA",
                "tracks": extra_tracks,
                "mean_cost": float(Jx),
                "episodes_done": int(episodes_done),
                "crashes": int(infox.get("crashes", 0)),
                "theta": {k: float(v) for k, v in best_theta.items()},
                "improved": False,
            })

            print(f"[EXTRA] mean_cost={Jx:.6g} crashes={infox.get('crashes',0)}/{infox.get('episodes',0)} | episodes={episodes_done}/{TOTAL_EPISODES_BUDGET}")

    finally:
        if roscore_p is not None:
            _kill_process_group(roscore_p)

        try:
            _append_checkpoint(episodes_done, best_cost, best_theta)
        except Exception:
            pass

    print("\n=== DONE ===")
    print(f"Episodes done: {episodes_done}/{TOTAL_EPISODES_BUDGET}")
    print(f"Trials done   : {min(trial_idx, TRIALS_TOTAL)}/{TRIALS_TOTAL} (each {EPISODES_PER_TRIAL} eps)")
    print(f"Best cost observed: {best_cost:.6g}")
    print("Best theta:")
    for k, v in best_theta.items():
        lo, hi = PARAM_BOUNDS[k]
        print(f"  {k}: {float(v):.6g}  (bounds [{lo}, {hi}])")
    print(f"\nBest JSON: {BEST_JSON_OUT}")
    print(f"Run log : {RUN_LOG_PATH}")
    print(f"Checkpoints: {CHECKPOINT_LOG_PATH} (every {CHECKPOINT_EVERY} episodes)")


if __name__ == "__main__":
    main()

