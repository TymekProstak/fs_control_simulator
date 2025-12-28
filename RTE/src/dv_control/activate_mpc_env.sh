#!/usr/bin/env bash
set -e

# Użycie:
#   source /sciezka/do/workspace/src/dv_control/activate_mpc_env.sh
# Skrypt jest "uniwersalny": nie zakłada konkretnej nazwy folderu workspacu.

# --- Ustal lokalizacje (bez twardych sciezek) ---
# BASH_SOURCE działa poprawnie przy "source".
SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"           # .../src/dv_control
WS_SRC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"                       # .../src
WS_ROOT_DIR="$(cd "$WS_SRC_DIR/.." && pwd)"                     # .../

# --- ROS ---
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  source "/opt/ros/noetic/setup.bash"
fi

# --- Workspace: source devel (jeśli istnieje) ---
# Catkin-tools zwykle ma: <ws>/devel/setup.bash
if [ -f "$WS_ROOT_DIR/devel/setup.bash" ]; then
  source "$WS_ROOT_DIR/devel/setup.bash"
else
  echo "[activate_mpc_env][WARN] Brak $WS_ROOT_DIR/devel/setup.bash (workspace może nie być zbudowany)" 1>&2
fi

# --- ACADOS (relatywnie do dv_control) ---
export ACADOS_SOURCE_DIR="$SCRIPT_DIR/External/acados"

# Preferuj install/ jeśli istnieje
if [ -d "$ACADOS_SOURCE_DIR/install" ]; then
  export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/install/lib:${LD_LIBRARY_PATH:-}"
  export C_INCLUDE_PATH="$ACADOS_SOURCE_DIR/install/include:${C_INCLUDE_PATH:-}"
fi

# bin/ (czasem potrzebne do narzędzi acados)
if [ -d "$ACADOS_SOURCE_DIR/bin" ]; then
  export PATH="$ACADOS_SOURCE_DIR/bin:$PATH"
fi

# --- Opcjonalnie: lokalne venv (relatywnie do root workspacu) ---
# U Ciebie jest: <ws>/src/mpc_env
if [ -f "$WS_SRC_DIR/mpc_env/bin/activate" ]; then
  source "$WS_SRC_DIR/mpc_env/bin/activate"
fi

# --- Helpery: uruchamianie komend ROS zawsze z tego workspacu ---
# Problem, który miałeś: roslaunch brał pakiety z innego overlay (np. ~/dv_ws).
# Te funkcje odpalają komendę w nowej powłoce, która najpierw source'uje
# /opt/ros/noetic i <ws>/devel/setup.bash.
roslaunch_ws() {
  local pkg="$1"; shift
  local file="$1"; shift
  bash -lc "source /opt/ros/noetic/setup.bash && \
    [ -f '$WS_ROOT_DIR/devel/setup.bash' ] && source '$WS_ROOT_DIR/devel/setup.bash'; \
    roslaunch '$pkg' '$file' $*"
}

rosrun_ws() {
  local pkg="$1"; shift
  local node="$1"; shift
  bash -lc "source /opt/ros/noetic/setup.bash && \
    [ -f '$WS_ROOT_DIR/devel/setup.bash' ] && source '$WS_ROOT_DIR/devel/setup.bash'; \
    rosrun '$pkg' '$node' $*"
}

# Skróty dla Twojego projektu (opcjonalne)
alias lem_sim='roslaunch_ws lem_simulator sim.launch'
alias dv_control_launch='roslaunch_ws dv_control control.launch'

echo "[activate_mpc_env] Workspace root: $WS_ROOT_DIR"
echo "[activate_mpc_env] ROS_MASTER_URI=${ROS_MASTER_URI:-unset}"
echo "[activate_mpc_env] ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR"
echo "[activate_mpc_env] Aliases: lem_sim, dv_control_launch (or use roslaunch_ws/rosrun_ws)"