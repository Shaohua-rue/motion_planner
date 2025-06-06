cmake_minimum_required(VERSION 3.11)
project(motion_planning)

# Set the C++ standard
add_definitions(-std=c++17)

# Set the build type
SET(CMAKE_BUILD_TYPE "Debug")


#  Set the output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 设置库文件输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#  Find dependencies
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)


# create a dummy library to link against
add_library(matplotlib_cpp INTERFACE)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 添加Python的include目录到matplotlib_cpp的接口包含路径
target_include_directories(matplotlib_cpp INTERFACE
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(matplotlib_cpp INTERFACE
    Python3::Python
    Python3::Module
)

find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE Python3::NumPy)
    # 如果需要，添加NumPy的include目录
    target_include_directories(matplotlib_cpp INTERFACE
        ${Python3_NumPy_INCLUDE_DIRS}
    )
else()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()



# Include directories
include_directories(
    "/usr/include/eigen3"
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/vehicle/include/
)

# Subdirectories
add_subdirectory(src/vehicle)
add_subdirectory(src/global_planner/graph_planner)
add_subdirectory(src/global_planner/sample_planner)