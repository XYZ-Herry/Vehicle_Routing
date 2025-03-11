cmake_minimum_required(VERSION 3.10)
project(delivery_system)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加源文件
set(SOURCES
    src/main.cpp
    src/common.cpp
    src/task_assigner.cpp
    src/genetic_algorithm.cpp
    src/path_optimizer.cpp
    src/solver.cpp
    src/dynamic_genetic.cpp
)

# 添加头文件目录
include_directories(include)

# 创建可执行文件
add_executable(delivery_system ${SOURCES}) 