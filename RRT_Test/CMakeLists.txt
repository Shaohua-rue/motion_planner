cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(RRTStarProject)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 如果你的 Eigen 库没有安装在标准路径，需要手动指定其包含目录
find_package(Eigen3 REQUIRED)

# 包含 Eigen 和 OpenCV 的头文件目录
include_directories(${OpenCV_INCLUDE_DIRS}
 "/usr/include/eigen3"
)

# 添加可执行文件
add_executable(rrt_smart_star src/rrt_smart_star.cpp)

# 链接 OpenCV 库
target_link_libraries(rrt_smart_star ${OpenCV_LIBS})

add_executable(rrtstar src/rrt_star.cpp)

# 链接 OpenCV 库
target_link_libraries(rrtstar ${OpenCV_LIBS})

add_executable(informed_rrt_smart src/informed_rrt_smart.cpp)

# 链接 OpenCV 库
target_link_libraries(informed_rrt_smart ${OpenCV_LIBS})
