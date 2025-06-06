cmake_minimum_required(VERSION 3.10)
project(vehicle)
message(STATUS "[${PROJECT_NAME}] Building....")
include_directories(
    include/
    ${Python3_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

add_library(vehicle SHARED 
  src/vehicle.cpp)

target_link_libraries(vehicle matplotlib_cpp)

