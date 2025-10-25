cmake_minimum_required(VERSION 3.7)  

project(lem_simulator)

add_compile_options(-std=c++17)

set(CMAKE_CXX_STANDARD 17)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")  # Dodanie optymalizacji

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  dv_interfaces

)


## Find external libraries
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs tf dv_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  
)
set( simSRC
  src_helpers/cone_detector.cpp
  src_helpers/cone_track.cpp
  src_helpers/double_track.cpp
  src_helpers/steering_system.cpp
  src_helpers/tire_model.cpp
  src_helpers/utiliities.cpp
)
add_executable(sim_loop_node  
    main.cpp
    sim_loop.cpp
    ${simSRC}  
)

target_link_libraries(sim_loop_node
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    m
)



