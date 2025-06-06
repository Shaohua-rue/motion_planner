cmake_minimum_required(VERSION 3.10)
message(STATUS "[${PROJECT_NAME}] Building....")
project(sample_planner)

include_directories(
    include/
)

add_executable(rrt_star
    src/rrt_star_main.cpp
    src/rrt_star.cpp
)
add_dependencies(rrt_star vehicle)
target_link_libraries(rrt_star
  vehicle
  fmt::fmt
  matplotlib_cpp
)

add_executable(rrt
    src/rrt_main.cpp
    src/rrt.cpp
)
add_dependencies(rrt vehicle)
target_link_libraries(rrt
  vehicle
  fmt::fmt
  matplotlib_cpp
)

add_executable(infored_rrt_star
    src/informed_rrt_star_main.cpp
    src/informed_rrt_star.cpp
)
add_dependencies(infored_rrt_star vehicle)
target_link_libraries(infored_rrt_star
  vehicle
  fmt::fmt
  matplotlib_cpp
)