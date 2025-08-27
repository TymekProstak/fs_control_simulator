cmake_minimum_required(VERSION 3.20)
project(fs_control_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- biblioteka / kod główny ---
add_library(fs_quazi_static
    source/engine.cpp   # wpisz tu wszystkie swoje .cpp
)
target_include_directories(fs_quazi_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(nlohmann_json REQUIRED)

target_link_libraries(fs_quazi_static PUBLIC nlohmann_json::nlohmann_json)


# --- GoogleTest + testy ---
find_package(GTest REQUIRED)
enable_testing()

add_executable(example_tests
    tests/example_test.cpp
)

target_link_libraries(example_tests PRIVATE
    GTest::gtest_main
    fs_quazi_static
)

include(GoogleTest)
gtest_discover_tests(example_tests)
