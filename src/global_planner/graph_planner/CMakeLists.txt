cmake_minimum_required(VERSION 3.10)
message(STATUS "[${PROJECT_NAME}] Building....")
project(graph_planner)

include_directories(
    include/
)

add_executable(graph_planner
    src/astar.cpp
    src/GraphSearchPlanner.cpp
)
add_dependencies(graph_planner vehicle)
target_link_libraries(graph_planner
  vehicle
  fmt::fmt
  matplotlib_cpp
)