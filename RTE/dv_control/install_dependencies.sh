#!/bin/bash
# ===========================================================
#  LOKALNA / PRZENOŚNA INSTALACJA ACADOS DO FOLDERA ./External/acados
#  + AUTOMATYCZNY EXPORT ŚCIEŻEK DO ~/.bashrc (przenośna wersja!)
# ===========================================================

# ---- WYZNACZ ŚCIEŻKĘ DO REPO ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EXTERNAL_DIR="$SCRIPT_DIR/External"
ACADOS_DIR="$EXTERNAL_DIR/acados"
INSTALL_DIR="$ACADOS_DIR/install"

echo ""
echo "=============================================="
echo "  ACADOS instalowany do:  $ACADOS_DIR"
echo "=============================================="

# ---- 1. utwórz External ----
mkdir -p "$EXTERNAL_DIR"

# ---- 2. wymagane pakiety systemowe ----
echo "[SYSTEM] Sprawdzam pakiety Eigen & JSON..."
sudo apt update

sudo apt install -y libeigen3-dev nlohmann-json3-dev \
    build-essential git cmake pkg-config

# ---- 3. pobranie / aktualizacja acados ----
if [ ! -d "$ACADOS_DIR" ]; then
  echo "[ACADOS] Klonuję repozytorium..."
  cd "$EXTERNAL_DIR" || exit
  git clone https://github.com/acados/acados.git --recursive
else
  echo "[ACADOS] Repozytorium już istnieje. Aktualizuję..."
  cd "$ACADOS_DIR" || exit
  git pull
  git submodule update --init --recursive
fi

# ---- 4. budowa ----
echo "[ACADOS] Budowanie i instalacja..."
cd "$ACADOS_DIR" || exit
mkdir -p build
cd build || exit

cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
  -DACADOS_WITH_QPOASES=ON \
  -DACADOS_WITH_OSQP=ON \
  -DACADOS_WITH_DAQP=ON

make -j$(nproc)
make install

# ---- 5. eksport ścieżek do .bashrc (PRZENOŚNA WERSJA) ----
echo ""
echo "=============================================="
echo "  Ustawiam zmienne środowiskowe w ~/.bashrc"
echo "=============================================="

BASHRC_UPDATE="\n# >>>>> ACADOS LOCAL SETUP >>>>>\n\
export ACADOS_SOURCE_DIR=\"\$HOME/sim_ws/src/dv_control/External/acados\"\n\
export LD_LIBRARY_PATH=\"\$ACADOS_SOURCE_DIR/install/lib:\$LD_LIBRARY_PATH\"\n\
export C_INCLUDE_PATH=\"\$ACADOS_SOURCE_DIR/install/include:\$C_INCLUDE_PATH\"\n\
export PATH=\"\$ACADOS_SOURCE_DIR/bin:\$PATH\"\n\
# <<<<< END ACADOS LOCAL SETUP <<<<<\n"

# **UWAGA** – dodaj TYLKO JEŚLI NIE ISTNIEJE
if ! grep -q "ACADOS LOCAL SETUP" ~/.bashrc; then
  echo -e "$BASHRC_UPDATE" >> ~/.bashrc
  echo "[OK] Dopisano do ~/.bashrc"
else
  echo "[INFO] .bashrc już zawiera wpis ACADOS – pomijam."
fi

# ---- 6. opcja automatycznego odświeżenia środowiska ----
echo ""
read -r -p "Czy chcesz od razu załadować zmienne? (y/n): " ANSW
if [[ "$ANSWER" == "y" || "$ANSWER" == "Y" ]]; then
    source ~/.bashrc
    echo "OK – zmienne środowiskowe załadowane."
else
    echo "Pamiętaj: przed użyciem ACADOS zrób: source ~/.bashrc"
fi

echo ""
echo "=============================================="
echo "    INSTALACJA ACADOS ZAKOŃCZONA"
echo "    Folder instalacji: $INSTALL_DIR"
echo ""
echo "    Dla Python:"
echo "      sys.path.append('$INSTALL_DIR/lib/python3.X/site-packages')"
echo ""
echo "    Dla CMakeLists.txt:"
echo "      set(ACADOS_DIR \${CMAKE_CURRENT_SOURCE_DIR}/External/acados/install)"
echo "=============================================="
