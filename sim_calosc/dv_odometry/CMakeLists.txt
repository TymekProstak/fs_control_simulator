cmake_minimum_required(VERSION 3.0.2)
project(dv_odometry)

add_compile_options(-std=c++17 -O2 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  dv_interfaces
)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json QUIET)

catkin_package(
  INCLUDE_DIRS include/complementary
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf2 tf2_ros dv_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/complementary/complementary.cpp
  src/complementary/wrapper.cpp
  src/complementary/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

if(TARGET nlohmann_json::nlohmann_json)
  target_link_libraries(${PROJECT_NAME}_node nlohmann_json::nlohmann_json)
endif()

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)