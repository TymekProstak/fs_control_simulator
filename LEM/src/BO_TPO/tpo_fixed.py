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

# needed before dv_control_node
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

# ======= BO (TPE) CONFIG =======
# Budżet epizodów: 1024
BO_TOTAL_EPISODES = 1024

# Każdy trial ma być oceniany na 3 epizodach, bez pruningu
EPISODES_PER_TRIAL = 3
BO_N_TRIALS = BO_TOTAL_EPISODES // EPISODES_PER_TRIAL          # 341 => 1023 epizody
BO_REMAINING_EPISODES = BO_TOTAL_EPISODES - BO_N_TRIALS * EPISODES_PER_TRIAL  # 1 epizod

# log-transform tylko dla wag MPC:
LOG_PARAMS = {
    "mpc.cost.Q_y",
    "mpc.cost.Q_psi",
    "mpc.cost.R_ddelta",
}

# ===== LHS startup (pierwsze triale zamiast random) =====
STARTUP_LHS_TRIALS = 25

# Clipy metryk (heurystyczne) – tylko metryki, NIE parametry
METRIC_CLIPS = {
    "vs_avg_mps": (0.0, 40.0),
    "ey_avg_m": (0.0, 3.0),
    "kappa_metric": (0.0, 5.0),
    "slip_angle_metric": (0.0, 1.2),
}

METRICS_WAIT_TIMEOUT_S = 10.0
EPISODE_HARD_TIMEOUT_MARGIN_S = 30.0

# ===== Reproducibility seed (kluczowe!) =====
GLOBAL_SEED = 123
random.seed(GLOBAL_SEED)

# ===== Checkpointing (ZGODNE MIĘDZY METODAMI) =====
CHECKPOINT_LOG_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__) or "."), "checkpoints.jsonl")
LOG_EVERY_EPISODES = 50

# ===== ROS master =====
ROS_MASTER_HOST = "127.0.0.1"
ROS_MASTER_PORT = 11311
START_ROSCORE_IF_NEEDED = True

# ===== ROS isolation (kluczowe!) =====
ROS_HOME = os.path.expanduser(f"~/.ros_{EXPERIMENT_ID}")
ROS_LOG_DIR = os.path.join(ROS_HOME, "log")

# =========================
#  GLOBAL STATE (runtime)
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

# ===== seeded track schedule (deterministyczne tory, osobny RNG) =====
def sample_k_unique_tracks_rng(rng: random.Random, k: int) -> List[int]:
    total = TRACK_INDEX_MAX - TRACK_INDEX_MIN + 1
    k = max(1, min(k, total))
    ids = list(range(TRACK_INDEX_MIN, TRACK_INDEX_MAX + 1))
    rng.shuffle(ids)
    return ids[:k]

# ===== deterministic track schedule per generation (for fair comparisons) =====
TRACKS_SEED = GLOBAL_SEED

def tracks_for_generation(generation_id: int, k: int) -> List[int]:
    total = TRACK_INDEX_MAX - TRACK_INDEX_MIN + 1
    kk = max(1, min(int(k), int(total)))
    rng = random.Random(int(TRACKS_SEED) + 999_983 + int(generation_id) * 1_000_003)
    ids = list(range(TRACK_INDEX_MIN, TRACK_INDEX_MAX + 1))
    rng.shuffle(ids)
    return ids[:kk]

# ===== Latin Hypercube Sampling in unit cube [0,1]^d (no numpy) =====
def _lhs_unit(rng: random.Random, n: int, d: int) -> List[List[float]]:
    # Each dimension: split into n strata, pick one point per stratum, then permute.
    n = int(n)
    d = int(d)
    u = [[0.0] * d for _ in range(n)]
    for j in range(d):
        strata = [(i + rng.random()) / float(n) for i in range(n)]
        rng.shuffle(strata)
        for i in range(n):
            u[i][j] = float(strata[i])
    return u

def _u_to_param_from_bounds(name: str, u: float) -> float:
    lo, hi = PARAM_BOUNDS[name]
    uu = float(u)
    if uu < 0.0:
        uu = 0.0
    if uu > 1.0:
        uu = 1.0
    if name in LOG_PARAMS:
        t = math.log(lo) + uu * (math.log(hi) - math.log(lo))
        x = math.exp(t)
    else:
        x = float(lo) + uu * (float(hi) - float(lo))
    return float(x)

def generate_lhs_startup_trials(seed: int, n_trials: int) -> List[Dict[str, float]]:
    keys = list(PARAM_BOUNDS.keys())
    d = len(keys)
    rng = random.Random(int(seed))
    U = _lhs_unit(rng, int(n_trials), int(d))
    trials: List[Dict[str, float]] = []
    for i in range(int(n_trials)):
        th: Dict[str, float] = {}
        for j, k in enumerate(keys):
            th[k] = _u_to_param_from_bounds(k, U[i][j])
        trials.append(th)
    return trials

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
    global EPISODES_DONE, NEXT_LOG_AT

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

    # --- episode accounting + checkpoints ---
    EPISODES_DONE += 1
    while EPISODES_DONE >= NEXT_LOG_AT:
        _append_checkpoint(NEXT_LOG_AT, BEST_COST_SO_FAR, BEST_THETA_SO_FAR)
        NEXT_LOG_AT += LOG_EVERY_EPISODES

    return res

def evaluate_theta_on_tracks(theta_x: Dict[str, float], track_ids: List[int], sim_time_s: int = DEFAULT_SIM_TIME_S) -> Tuple[float, Dict[str, Any]]:
    costs = []
    crashes = 0
    for tid in track_ids:
        res = run_one_episode(theta_x, tid, sim_time_s)
        costs.append(res.cost)
        if res.crashed:
            crashes += 1
    mean_cost = sum(costs) / max(1, len(costs))
    info = {"tracks": track_ids, "crashes": crashes, "episodes": len(track_ids), "mean_cost": mean_cost}
    return mean_cost, info

# --------- TPE sampling helpers ---------

def _suggest_param(trial, name: str, lo: float, hi: float) -> float:
    if name in LOG_PARAMS:
        return float(trial.suggest_float(name, lo, hi, log=True))
    else:
        return float(trial.suggest_float(name, lo, hi))

def suggest_theta_x(trial) -> Dict[str, float]:
    theta: Dict[str, float] = {}
    for k, (lo, hi) in PARAM_BOUNDS.items():
        theta[k] = _suggest_param(trial, k, lo, hi)
    return theta

def main():
    global BEST_COST_SO_FAR, BEST_THETA_SO_FAR, EPISODES_DONE, NEXT_LOG_AT

    try:
        import optuna
    except ImportError:
        print("Brak biblioteki 'optuna'. Zainstaluj: pip install optuna", file=sys.stderr)
        sys.exit(1)

    try:
        _must_exist(SIM_LAUNCH, "SIM_LAUNCH")
        _must_exist(CTRL_LAUNCH, "CTRL_LAUNCH")
        _must_exist(CONTROL_PARAM_JSON, "CONTROL_PARAM_JSON")
    except Exception as e:
        print("[FATAL]", e, file=sys.stderr)
        sys.exit(2)

    # Seed everything deterministically
    random.seed(GLOBAL_SEED)
    try:
        import numpy as np
        np.random.seed(GLOBAL_SEED)
    except Exception:
        pass

    # osobny deterministyczny RNG tylko do torów
    rng_tracks = random.Random(GLOBAL_SEED + 999)

    # reset liczników checkpointów (żeby zawsze startował identycznie)
    EPISODES_DONE = 0
    NEXT_LOG_AT = LOG_EVERY_EPISODES
    BEST_COST_SO_FAR = float("inf")
    BEST_THETA_SO_FAR = {}

    # checkpoint startowy (0) - spójne z CMA
    try:
        _append_checkpoint(0, BEST_COST_SO_FAR, BEST_THETA_SO_FAR)
    except Exception:
        pass

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

    # Optuna sampler seed = GLOBAL_SEED for full repeatability
    sampler = optuna.samplers.TPESampler(
        seed=GLOBAL_SEED,
        n_startup_trials=STARTUP_LHS_TRIALS,
        n_ei_candidates=32,
        multivariate=True,
        constant_liar=True,
    )

    # No pruning
    pruner = optuna.pruners.NopPruner()
    study = optuna.create_study(direction="minimize", sampler=sampler, pruner=pruner)

    # ===== LHS startup: first STARTUP_LHS_TRIALS trials are fixed LHS points (deterministic) =====
    try:
        lhs_trials = generate_lhs_startup_trials(GLOBAL_SEED + 4242, STARTUP_LHS_TRIALS)
        for th in lhs_trials:
            study.enqueue_trial({k: float(v) for k, v in th.items()})
        print(f"[LHS] Enqueued {len(lhs_trials)} startup trials (LHS).")
    except Exception as e:
        print(f"[LHS] WARN: cannot enqueue LHS startup trials: {e}", file=sys.stderr)

    best_out_path = os.path.join(os.path.dirname(CONTROL_PARAM_JSON), "control_param_best_from_tpe.json")

    print(f"[BO] Target total episodes = {BO_TOTAL_EPISODES}")
    print(f"[BO] Trials = {BO_N_TRIALS} (each {EPISODES_PER_TRIAL} episodes => {BO_N_TRIALS*EPISODES_PER_TRIAL} episodes)")
    print(f"[BO] Remaining episodes after trials = {BO_REMAINING_EPISODES} (extra validation)\n")
    print(f"[CKPT] Every {LOG_EVERY_EPISODES} episodes -> {CHECKPOINT_LOG_PATH}\n")

    def objective(trial: "optuna.trial.Trial") -> float:
        global BEST_COST_SO_FAR, BEST_THETA_SO_FAR

        theta_x = suggest_theta_x(trial)
        # ===== generation-indexed tracks (same for all candidates inside that generation) =====
        if trial.number < STARTUP_LHS_TRIALS:
            generation_id = 0
        else:
            generation_id = (trial.number - STARTUP_LHS_TRIALS) + 1
        tracks = tracks_for_generation(generation_id, EPISODES_PER_TRIAL)
        J, info = evaluate_theta_on_tracks(theta_x, tracks)

        # update best-so-far (for checkpointing)
        if J < BEST_COST_SO_FAR:
            BEST_COST_SO_FAR = float(J)
            BEST_THETA_SO_FAR = {k: float(v) for k, v in theta_x.items()}

        print(f"[TRIAL {trial.number}] gen={generation_id} tracks={tracks} mean_cost={J:.6g} crashes={info.get('crashes',0)}/{info.get('episodes',0)}")
        return J

    try:
        study.optimize(objective, n_trials=BO_N_TRIALS, gc_after_trial=True, show_progress_bar=False)
    finally:
        # Save best params json (Optuna best by mean over 3 tracks)
        try:
            best = study.best_trial
            data = _load_json(CONTROL_PARAM_JSON)
            for path, val in best.params.items():
                _set_json_path(data, path, float(val))
            _save_json_atomic(best_out_path, data)
            print(f"\n[BEST] Saved best params -> {best_out_path}")
        except Exception as e:
            print(f"\n[BEST] WARN: cannot save best json: {e}", file=sys.stderr)

        # Extra episodes to hit exactly 1024 (does NOT affect learning)
        if BO_REMAINING_EPISODES > 0 and len(study.trials) > 0:
            try:
                best_theta = {k: float(v) for k, v in study.best_trial.params.items()}
                extra_tracks = tracks_for_generation((BO_N_TRIALS - STARTUP_LHS_TRIALS) + 2, BO_REMAINING_EPISODES)
                print(f"\n[EXTRA] Running {BO_REMAINING_EPISODES} extra episodes on tracks={extra_tracks}")
                J_extra, info_extra = evaluate_theta_on_tracks(best_theta, extra_tracks)
                # update best-so-far if it improved (rare but possible)
                if J_extra < BEST_COST_SO_FAR:
                    BEST_COST_SO_FAR = float(J_extra)
                    BEST_THETA_SO_FAR = {k: float(v) for k, v in best_theta.items()}
                print(f"[EXTRA] mean_cost={J_extra:.6g} crashes={info_extra.get('crashes',0)}/{info_extra.get('episodes',0)}")
            except Exception as e:
                print(f"[EXTRA] WARN: cannot run extra episodes: {e}", file=sys.stderr)

        if roscore_p is not None:
            _kill_process_group(roscore_p)

    print("\n=== DONE ===")
    print(f"Best value (mean over 3 tracks): {study.best_value:.6g}")
    print("Best params:")
    for k, v in study.best_params.items():
        lo, hi = PARAM_BOUNDS[k]
        print(f"  {k}: {float(v):.6g}  (bounds [{lo}, {hi}])")

    print(f"\nBest json saved at: {best_out_path}")
    print(f"Checkpoints saved at: {CHECKPOINT_LOG_PATH}")
    print(f"\nTotal episodes budget: {BO_TOTAL_EPISODES} = {BO_N_TRIALS*EPISODES_PER_TRIAL} (trials) + {BO_REMAINING_EPISODES} (extra)")

if __name__ == "__main__":
    main()

