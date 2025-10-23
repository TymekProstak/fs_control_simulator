cmake_minimum_required(VERSION 3.10)
project(test_tire_model)

# Ustawienia kompilatora
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Dodanie bibliotek
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Pliki źródłowe
add_executable(test_tire_model
    test_tire_model_time.cpp
    tire_model.cpp
    utiliities.cpp
)

# Dołączanie katalogów nagłówkowych
target_include_directories(test_tire_model PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(test_tire_model PRIVATE ${nlohmann_json_INCLUDE_DIRS})

# Linkowanie bibliotek
target_link_libraries(test_tire_model PRIVATE nlohmann_json::nlohmann_json)