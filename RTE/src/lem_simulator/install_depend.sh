#!/bin/bash
set -e

# --- Sprawdzenie i instalacja Eigen ---
if [ ! -d "/usr/include/eigen3" ]; then
  echo "Eigen3 nie znaleziono. Instalacja..."
  sudo apt update
  sudo apt install -y libeigen3-dev
else
  echo "Eigen3 już jest zainstalowany."
fi

# --- Sprawdzenie i instalacja nlohmann/json ---
if [ ! -f "/usr/include/nlohmann/json.hpp" ]; then
  echo "nlohmann/json nie znaleziono. Instalacja..."
  sudo apt update
  sudo apt install -y nlohmann-json3-dev
else
  echo "nlohmann/json już jest zainstalowany."
fi

echo "Gotowe. Możesz teraz używać:"
echo ""
echo "  find_package(Eigen3 REQUIRED)"
echo "  find_package(nlohmann_json REQUIRED)"
echo ""
echo "i w kodzie:"
echo "#include <Eigen/Dense>"
echo "#include <nlohmann/json.hpp>"
