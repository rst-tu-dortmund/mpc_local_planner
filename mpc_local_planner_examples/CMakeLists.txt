cmake_minimum_required(VERSION 3.5)
project(mpc_local_planner_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch cfg maps stage
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
