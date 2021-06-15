cmake_minimum_required(VERSION 3.5)
project(mpc_local_planner_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(mpc_local_planner_msgs
  "msg/OptimalControlResult.msg"
  "msg/StateFeedback.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
