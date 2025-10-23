cmake_minimum_required(VERSION 3.10)
project(fs_control_simulator)

# Ustawienia kompilatora
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Znajdź wymagane biblioteki
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Dodaj plik wykonywalny dla testu cone_detector
add_executable(test_cone_detector
    test_cone_detector.cpp
    cone_detector.cpp
    utiliities.cpp
)

# Dołącz katalogi nagłówkowe
target_include_directories(test_cone_detector PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(test_cone_detector PRIVATE nlohmann_json::nlohmann_json)