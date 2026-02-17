#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lem_eval.py
===========

Ten skrypt jest "czarną skrzynką" f(theta):
- bierze słownik parametrów theta (np. {"mpc.cost.Q_y": 80, ...})
- (opcjonalnie) ustawia też parametry LEM w params_default_lem.json
- odpala roslaunch symulacji + roslaunch kontrolera
- czeka aż symulacja się skończy
- czyta lem_simulator/logs/run_default_metrics.csv
- liczy koszt J (do minimalizacji) i zwraca wynik jako JSON

Bayesian Optimization NIE siedzi tutaj — BO będzie w bayes_tune.py.
To rozdzielenie jest celowe: eval ma być stabilny i prosty.
"""

import os
import sys
import json
import time
import math
import signal
import socket
import random
import argparse
import subprocess
from dataclasses import dataclass
from typing import Dict, Any, Optional, List, Tuple


# -------------------------
# JSON helpers
# -------------------------

def set_json_path(d: Dict[str, Any], path: str, value: Any) -> None:
    keys = path.split(".")
    cur = d
    for k in keys[:-1]:
        if k not in cur or not isinstance(cur[k], dict):
            cur[k] = {}
        cur = cur[k]
    cur[keys[-1]] = value

def get_json_path(d: Dict[str, Any], path: str) -> Any:
    keys = path.split(".")
    cur: Any = d
    for k in keys:
        cur = cur[k]
    return cur

def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return json.load(f)

def save_json_atomic(path: str, data: Dict[str, Any]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


# -------------------------
# ROS process helpers
# -------------------------

def rosmaster_is_up(host: str, port: int, timeout: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False

def popen_roscore() -> subprocess.Popen:
    p = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
        preexec_fn=os.setsid,
        text=True
    )
    return p

def popen_roslaunch(launch_file: str, launch_args: Dict[str, str], extra_env: Optional[Dict[str, str]] = None) -> subprocess.Popen:
    cmd = ["roslaunch", launch_file] + [f"{k}:={v}" for k, v in launch_args.items()]
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    p = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        text=True
    )
    p._cmd = cmd  # debug only
    return p

def kill_process_group(p: Optional[subprocess.Popen], sig=signal.SIGINT, timeout_s: float = 5.0) -> None:
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
            return
        time.sleep(0.1)

    try:
        if pgid is not None:
            os.killpg(pgid, signal.SIGKILL)
        else:
            p.kill()
    except Exception:
        pass


# -------------------------
# Metrics CSV parsing
# -------------------------

def parse_metrics_csv(path: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    if not os.path.exists(path):
        return out

    with open(path, "r") as f:
        lines = f.read().splitlines()

    if not lines:
        return out

    # expected CSV format: header, then key,value lines
    for line in lines[1:]:
        if not line.strip():
            continue
        if "," not in line:
            continue
        k, v = line.split(",", 1)
        k = k.strip()
        v = v.strip().strip('"')

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

def wait_for_metrics_stable(path: str, timeout_s: float) -> bool:
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


# -------------------------
# Objective (cost) computation
# -------------------------

def pick_speed_key(metrics: Dict[str, Any], candidates: List[str]) -> Optional[str]:
    for k in candidates:
        if k in metrics:
            return k
    return None

def compute_cost(
    metrics: Dict[str, Any],
    *,
    sim_time_s: int,
    speed_key_candidates: List[str],
    weights: Dict[str, float],
    crash_penalty: float,
    crash_time_penalty: float
) -> float:
    """
    Zwracamy J do MINIMALIZACJI.

    - Gdy nie ma crasha:
        J = sum_k w_k * metric_k
      (np. w_speed < 0 => większa prędkość zmniejsza J)

    - Gdy crash:
        J = crash_penalty + crash_time_penalty*(sim_time - crash_time)

    To jest spójne z praktyką z paperów o crash constraints:
    crash daje "złą" obserwację, ale my jeszcze dodatkowo
    w BO będziemy używać VDP (adaptive penalty).
    """
    crashed = int(metrics.get("crashed", 0)) == 1
    if crashed:
        crash_time = float(metrics.get("crash_time_s", -1.0))
        return crash_penalty + crash_time_penalty * max(0.0, sim_time_s - max(0.0, crash_time))

    # jeśli w metrykach jest vs_avg_mpc zamiast vs_avg_mps, to mapujemy pod wagę speed:
    speed_key = pick_speed_key(metrics, speed_key_candidates)
    J = 0.0

    for k, w in weights.items():
        # allow "vs_avg_mps" weight even if metrics uses different speed key
        kk = k
        if k == "vs_avg_mps" and speed_key is not None:
            kk = speed_key

        try:
            val = float(metrics.get(kk, 0.0))
        except Exception:
            val = 0.0
        J += float(w) * val

    return float(J)


# -------------------------
# Episode runner
# -------------------------

@dataclass
class EpisodeResult:
    cost: float
    metrics: Dict[str, Any]
    track_id: int
    crashed: bool

def sample_track(track_min: int, track_max: int) -> int:
    return random.randint(track_min, track_max)

def apply_param_set_with_backup(json_path: str, theta: Dict[str, float]) -> Dict[str, Any]:
    """
    Zwraca oryginalną zawartość json, żeby dało się ją przywrócić.
    """
    original = load_json(json_path)
    data = json.loads(json.dumps(original))  # deep-ish copy

    for k, v in theta.items():
        set_json_path(data, k, float(v))

    save_json_atomic(json_path, data)
    return original

def apply_overrides_with_backup(json_path: str, overrides: Dict[str, Any]) -> Dict[str, Any]:
    original = load_json(json_path)
    data = json.loads(json.dumps(original))

    for k, v in overrides.items():
        set_json_path(data, k, v)

    save_json_atomic(json_path, data)
    return original

def run_one_episode(
    *,
    theta: Dict[str, float],
    track_id: int,
    sim_time_s: int,

    sim_launch: str,
    ctrl_launch: str,

    control_param_json: str,
    lem_params_json: Optional[str],
    lem_params_overrides: Optional[Dict[str, Any]],
    apply_lem_overrides_each_episode: bool,

    metrics_csv: str,
    metrics_wait_timeout_s: float,
    episode_hard_timeout_margin_s: float,

    ins_mode: str,
    low_level_controlers: str,

    acados_lib: str,

    speed_key_candidates: List[str],
    weights: Dict[str, float],
    crash_penalty: float,
    crash_time_penalty: float,

    restore_files_after_episode: bool = True
) -> EpisodeResult:

    # --- backup & apply dv_control params ---
    ctrl_backup = apply_param_set_with_backup(control_param_json, theta)

    # --- optional: apply LEM overrides (once or each episode) ---
    lem_backup = None
    if lem_params_json and lem_params_overrides:
        if apply_lem_overrides_each_episode:
            lem_backup = apply_overrides_with_backup(lem_params_json, lem_params_overrides)

    # --- remove old metrics ---
    try:
        if os.path.exists(metrics_csv):
            os.remove(metrics_csv)
    except Exception:
        pass

    sim_args = {
        "sim_time": str(sim_time_s),
        "track_id": str(track_id),
        "ins_mode": str(ins_mode),
        "low_level_controlers": str(low_level_controlers)
    }

    sim_p = popen_roslaunch(sim_launch, sim_args)
    ctrl_env = {"LD_LIBRARY_PATH": f"{acados_lib}:" + os.environ.get("LD_LIBRARY_PATH", "")}
    ctrl_p = popen_roslaunch(ctrl_launch, {}, extra_env=ctrl_env)

    time.sleep(1.0)

    # early launch failure handling
    if sim_p.poll() is not None:
        metrics = {"crashed": 1, "crash_reason": f"sim_launch_failed_rc={int(sim_p.returncode)}", "crash_time_s": -1}
        cost = compute_cost(
            metrics, sim_time_s=sim_time_s,
            speed_key_candidates=speed_key_candidates,
            weights=weights, crash_penalty=crash_penalty, crash_time_penalty=crash_time_penalty
        )
        kill_process_group(sim_p)
        kill_process_group(ctrl_p)
        if restore_files_after_episode:
            save_json_atomic(control_param_json, ctrl_backup)
            if lem_backup is not None and lem_params_json:
                save_json_atomic(lem_params_json, lem_backup)
        return EpisodeResult(cost=cost, metrics=metrics, track_id=track_id, crashed=True)

    if ctrl_p.poll() is not None:
        metrics = {"crashed": 1, "crash_reason": f"ctrl_launch_failed_rc={int(ctrl_p.returncode)}", "crash_time_s": -1}
        cost = compute_cost(
            metrics, sim_time_s=sim_time_s,
            speed_key_candidates=speed_key_candidates,
            weights=weights, crash_penalty=crash_penalty, crash_time_penalty=crash_time_penalty
        )
        kill_process_group(sim_p)
        kill_process_group(ctrl_p)
        if restore_files_after_episode:
            save_json_atomic(control_param_json, ctrl_backup)
            if lem_backup is not None and lem_params_json:
                save_json_atomic(lem_params_json, lem_backup)
        return EpisodeResult(cost=cost, metrics=metrics, track_id=track_id, crashed=True)

    # wait until sim ends or hard timeout
    t0 = time.time()
    while True:
        if sim_p.poll() is not None:
            break
        if time.time() - t0 > sim_time_s + episode_hard_timeout_margin_s:
            break
        time.sleep(0.2)

    kill_process_group(sim_p)
    kill_process_group(ctrl_p)

    ok = wait_for_metrics_stable(metrics_csv, metrics_wait_timeout_s)
    if not ok:
        metrics = {"crashed": 1, "crash_reason": "no_metrics_file", "crash_time_s": -1}
        cost = compute_cost(
            metrics, sim_time_s=sim_time_s,
            speed_key_candidates=speed_key_candidates,
            weights=weights, crash_penalty=crash_penalty, crash_time_penalty=crash_time_penalty
        )
        if restore_files_after_episode:
            save_json_atomic(control_param_json, ctrl_backup)
            if lem_backup is not None and lem_params_json:
                save_json_atomic(lem_params_json, lem_backup)
        return EpisodeResult(cost=cost, metrics=metrics, track_id=track_id, crashed=True)

    metrics = parse_metrics_csv(metrics_csv)
    cost = compute_cost(
        metrics, sim_time_s=sim_time_s,
        speed_key_candidates=speed_key_candidates,
        weights=weights, crash_penalty=crash_penalty, crash_time_penalty=crash_time_penalty
    )
    crashed = int(metrics.get("crashed", 0)) == 1

    if restore_files_after_episode:
        save_json_atomic(control_param_json, ctrl_backup)
        if lem_backup is not None and lem_params_json:
            save_json_atomic(lem_params_json, lem_backup)

    return EpisodeResult(cost=cost, metrics=metrics, track_id=track_id, crashed=crashed)


# -------------------------
# CLI
# -------------------------

def expand(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="tuning_config.json")
    ap.add_argument("--theta_json", required=True, help="JSON string: {\"mpc.cost.Q_y\": 80.0, ...}")
    ap.add_argument("--track_id", type=int, default=-1)
    ap.add_argument("--print_json_only", action="store_true")
    args = ap.parse_args()

    cfg = load_json(expand(args.config))

    wroot = expand(cfg["paths"]["workspace_root"])
    sim_launch = os.path.join(wroot, cfg["paths"]["sim_launch_rel"])
    ctrl_launch = os.path.join(wroot, cfg["paths"]["ctrl_launch_rel"])
    control_param_json = os.path.join(wroot, cfg["paths"]["control_param_json_rel"])
    lem_params_json = os.path.join(wroot, cfg["paths"]["lem_params_json_rel"]) if cfg["paths"].get("lem_params_json_rel") else None
    metrics_csv = os.path.join(wroot, cfg["paths"]["metrics_csv_rel"])
    acados_lib = expand(cfg["paths"]["acados_lib"])

    theta = json.loads(args.theta_json)

    # apply LEM overrides once (if configured and not "each episode")
    lem_over = cfg.get("lem_params_overrides", {})
    lem_enabled = bool(lem_over.get("enabled", False))
    apply_each = bool(lem_over.get("apply_each_episode", False))
    lem_overrides_vals = lem_over.get("values", {}) if lem_enabled else None

    # if "apply_each_episode" is False, apply once here
    lem_once_backup = None
    if lem_enabled and (not apply_each) and lem_params_json and lem_overrides_vals:
        lem_once_backup = apply_overrides_with_backup(lem_params_json, lem_overrides_vals)

    # optional: start roscore
    ros_cfg = cfg.get("ros", {})
    host = ros_cfg.get("ros_master_host", "127.0.0.1")
    port = int(ros_cfg.get("ros_master_port", 11311))
    start_roscore = bool(ros_cfg.get("start_roscore_if_needed", True))
    roscore_p = None
    if start_roscore and (not rosmaster_is_up(host, port)):
        roscore_p = popen_roscore()
        time.sleep(1.0)

    try:
        track_cfg = cfg["tracks"]
        if args.track_id >= 0:
            track_id = int(args.track_id)
        else:
            track_id = sample_track(int(track_cfg["track_index_min"]), int(track_cfg["track_index_max"]))

        sim_cfg = cfg["sim"]
        obj_cfg = cfg["objective"]

        res = run_one_episode(
            theta=theta,
            track_id=track_id,
            sim_time_s=int(sim_cfg["default_sim_time_s"]),

            sim_launch=sim_launch,
            ctrl_launch=ctrl_launch,

            control_param_json=control_param_json,
            lem_params_json=lem_params_json if lem_enabled else None,
            lem_params_overrides=lem_overrides_vals if lem_enabled else None,
            apply_lem_overrides_each_episode=apply_each,

            metrics_csv=metrics_csv,
            metrics_wait_timeout_s=float(sim_cfg.get("metrics_wait_timeout_s", 10.0)),
            episode_hard_timeout_margin_s=float(sim_cfg.get("episode_hard_timeout_margin_s", 30.0)),

            ins_mode=str(sim_cfg.get("ins_mode", "kalman")),
            low_level_controlers=str(sim_cfg.get("low_level_controlers", "false")),

            acados_lib=acados_lib,

            speed_key_candidates=list(obj_cfg.get("speed_key_candidates", ["vs_avg_mps"])),
            weights=dict(obj_cfg["weights"]),
            crash_penalty=float(obj_cfg["crash_penalty"]),
            crash_time_penalty=float(obj_cfg["crash_time_penalty"]),

            restore_files_after_episode=True
        )

        out = {
            "track_id": res.track_id,
            "crashed": bool(res.crashed),
            "cost": float(res.cost),
            "metrics": res.metrics,
            "theta": theta
        }

        if args.print_json_only:
            print(json.dumps(out))
        else:
            reason = out["metrics"].get("crash_reason", "")
            print(f"[lem_eval] track={res.track_id} crashed={1 if res.crashed else 0} cost={res.cost:.6g} reason={reason}")
            print(json.dumps(out))

        return 0

    finally:
        if roscore_p is not None:
            kill_process_group(roscore_p)

        # restore LEM overrides if applied once
        if lem_once_backup is not None and lem_params_json:
            try:
                save_json_atomic(lem_params_json, lem_once_backup)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())