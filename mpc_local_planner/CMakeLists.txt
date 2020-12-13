cmake_minimum_required(VERSION 3.1)
project(mpc_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  costmap_converter
  dynamic_reconfigure
  roscpp
  geometry_msgs
  interactive_markers
  nav_core
  nav_msgs
  mbf_costmap_core
  mbf_msgs
  mpc_local_planner_msgs
  pluginlib
  std_msgs
  teb_local_planner
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# Find control_box_rst
# TODO: Currently, we use QUIET rather than REQUIRED to avoid warnings
#       as some components depend on each other (circularly)
#       which is currently not handled well by the exported cmake config
find_package(control_box_rst QUIET COMPONENTS
  core
  communication
  controllers
  numerics
  systems
  optimization
  optimal_control
  systems
  PATH_SUFFIXES share/control_box_rst control_box_rst
  PATHS ${CMAKE_BINARY_DIR} /usr/local
)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
# if (EXISTS "FindEigen3.cmake")
find_package(Eigen3 REQUIRED)

find_package(osqp QUIET)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(EXTERNAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


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
#   std_msgs  # Or other packages containing msgs
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

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/mpc_collision.cfg
  cfg/mpc_controller.cfg
  cfg/mpc_footprint.cfg
)

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
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES mpc_local_planner_utils mpc_local_planner_optimal_control mpc_local_planner
  CATKIN_DEPENDS roscpp mpc_local_planner_msgs control_box_rst teb_local_planner dynamic_reconfigure
  # DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(mpc_local_planner_utils
   src/utils/publisher.cpp
   src/utils/conversion.cpp
   src/utils/time_series_se2.cpp
)

add_library(mpc_local_planner_optimal_control
   src/optimal_control/full_discretization_grid_base_se2.cpp
   src/optimal_control/finite_differences_grid_se2.cpp
   src/optimal_control/finite_differences_variable_grid_se2.cpp
   src/optimal_control/stage_inequality_se2.cpp
   src/optimal_control/quadratic_cost_se2.cpp
   src/optimal_control/final_state_conditions_se2.cpp
   src/optimal_control/min_time_via_points_cost.cpp
)

add_library(mpc_local_planner
   src/controller.cpp
   src/mpc_local_planner_ros.cpp
)

# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(mpc_local_planner ${PROJECT_NAME}_gencfg)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(test_mpc_optim_node src/test_mpc_optim_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(mpc_local_planner_utils corbo_core corbo_systems)
target_link_libraries(mpc_local_planner_utils
  corbo_core
  corbo_systems
)

target_link_libraries(mpc_local_planner_optimal_control
  corbo_optimal_control
)

target_link_libraries(mpc_local_planner
  mpc_local_planner_utils
  mpc_local_planner_optimal_control
  corbo_controllers
  ${catkin_LIBRARIES}
)

 target_link_libraries(test_mpc_optim_node
   mpc_local_planner
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/plot_optimal_control_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS mpc_local_planner mpc_local_planner_utils mpc_local_planner_optimal_control
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS test_mpc_optim_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  mpc_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch cfg scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mpc_local_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
