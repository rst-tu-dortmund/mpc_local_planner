cmake_minimum_required(VERSION 3.5)
project(mpc_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util)
find_package(nav_2d_utils)
find_package(costmap_converter REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mpc_local_planner_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(p90902_controller_msgs)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teb_local_planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")

find_package(G2O REQUIRED)

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

set(EXTERNAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} ${teb_local_planner_INCLUDE_DIRS} ${G2O_INCLUDE_DIR}/../)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(mpc_local_planner_utils SHARED
   src/utils/publisher.cpp
   src/utils/conversion.cpp
   src/utils/time_series_se2.cpp
)

add_library(mpc_local_planner_optimal_control SHARED
   src/optimal_control/full_discretization_grid_base_se2.cpp
   src/optimal_control/finite_differences_grid_se2.cpp
   src/optimal_control/finite_differences_variable_grid_se2.cpp
   src/optimal_control/stage_inequality_se2.cpp
   src/optimal_control/quadratic_cost_se2.cpp
   src/optimal_control/final_state_conditions_se2.cpp
   src/optimal_control/min_time_via_points_cost.cpp
)

add_library(mpc_local_planner SHARED
   src/controller.cpp
   src/mpc_local_planner_ros.cpp
   src/mpc_config.cpp
)

set(ament_dependencies
    builtin_interfaces
    nav2_core
    nav2_costmap_2d
    nav2_util
    nav_2d_utils
    costmap_converter
    rclcpp
    geometry_msgs
    interactive_markers
    nav_msgs
    mpc_local_planner_msgs
    pluginlib
    p90902_controller_msgs
    rosidl_runtime_cpp
    std_msgs
    teb_local_planner
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)
ament_target_dependencies(mpc_local_planner
   ${ament_dependencies}
)

ament_target_dependencies(mpc_local_planner_optimal_control
   ${ament_dependencies}
)

ament_target_dependencies(mpc_local_planner_utils
   ${ament_dependencies}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(test_mpc_optim_node src/test_mpc_optim_node.cpp)

#ament_target_dependencies(test_mpc_optim_node
#   ${ament_dependencies}
#)

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
)

# target_link_libraries(test_mpc_optim_node
#   mpc_local_planner
# )

install(PROGRAMS
  scripts/plot_optimal_control_results.py
  DESTINATION lib/${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
install(TARGETS mpc_local_planner mpc_local_planner_utils mpc_local_planner_optimal_control
   DESTINATION lib
)
#install(TARGETS test_mpc_optim_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  mpc_local_planner_plugin.xml
  DESTINATION share
)

install(DIRECTORY
  launch cfg scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mpc_local_planner)
ament_export_dependencies(${ament_dependencies})
pluginlib_export_plugin_description_file(nav2_core mpc_local_planner_plugin.xml)

ament_package()

