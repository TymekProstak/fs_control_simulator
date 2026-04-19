cmake_minimum_required(VERSION 3.5)
project(dv_path_planning)

add_compile_options(-std=c++17)
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  dv_interfaces
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB path_planning_SRC 
    src/path_planner/PathPlanner.cpp
    src/cone_chain/Cone.cpp
    src/common/BolideDescriptor.cpp
    src/common/Geometry.cpp
    src/rviz/RvizConesMarkers.cpp
    src/rviz/RvizMarkers.cpp
)

add_executable(dv_path_planning_discovery_node
        src/path_planner/DiscoveryPathPlanner.cpp
        src/path_planning_discovery_node.cpp
        ${path_planning_SRC}
        src/cone_chain/ConeLookup.cpp
)
target_link_libraries(
  dv_path_planning_discovery_node ${catkin_LIBRARIES}
)
add_dependencies(dv_path_planning_discovery_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(dv_path_planning_trackdrive_node
        src/path_planner/TrackdrivePathPlanner.cpp
        src/path_planning_trackdrive_node.cpp
        ${path_planning_SRC}
        src/cone_chain/ConeLookup.cpp
)
target_link_libraries(
  dv_path_planning_trackdrive_node ${catkin_LIBRARIES}
)
add_dependencies(dv_path_planning_trackdrive_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    ${PROJECT_NAME}-test
    test/testVec2.cpp
    test/testConeDesc.cpp
    test/testBolideDesc.cpp
    test/testGeometry.cpp
    ${path_planning_SRC}
  )

  target_link_libraries(
    ${PROJECT_NAME}-test 
    ${catkin_LIBRARIES}
    GTest::gtest_main
  )
endif()
