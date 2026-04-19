cmake_minimum_required(VERSION 3.5)
project(dv_graphslam)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_ros
  visualization_msgs
  eigen_conversions
  dv_interfaces
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

find_path(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  PATHS
    /opt/ros/noetic/include
    /usr/include
    /usr/local/include
)

find_library(G2O_CORE_LIBRARY NAMES g2o_core
  PATHS
    /opt/ros/noetic/lib
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

find_library(G2O_STUFF_LIBRARY NAMES g2o_stuff
  PATHS
    /opt/ros/noetic/lib
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

find_library(G2O_SOLVER_EIGEN_LIBRARY NAMES g2o_solver_eigen
  PATHS
    /opt/ros/noetic/lib
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

find_library(G2O_TYPES_SLAM2D_LIBRARY NAMES g2o_types_slam2d
  PATHS
    /opt/ros/noetic/lib
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

if(NOT G2O_INCLUDE_DIR OR
   NOT G2O_CORE_LIBRARY OR
   NOT G2O_STUFF_LIBRARY OR
   NOT G2O_SOLVER_EIGEN_LIBRARY OR
   NOT G2O_TYPES_SLAM2D_LIBRARY)
  message(FATAL_ERROR "g2o was not found. Check where libg2o is installed.")
endif()

set(G2O_LIBRARIES
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN_LIBRARY}
  ${G2O_TYPES_SLAM2D_LIBRARY}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    pcl_ros
    visualization_msgs
    eigen_conversions
    dv_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(dv_graphslam_node
  src/graphslam_node.cpp
  src/GraphSLAM.cpp
  src/GraphManager.cpp
)

target_link_libraries(dv_graphslam_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_LIBRARIES}
  Eigen3::Eigen
)

add_dependencies(dv_graphslam_node
  ${catkin_EXPORTED_TARGETS}
)