cmake_minimum_required(VERSION 3.30.1)
project(Rasterizer)

# 设置 C++ 标准为 C++23
set(CMAKE_CXX_STANDARD 23)

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)

# 包含 OpenCV 头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

# 链接 OpenCV 库
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})