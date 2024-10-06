cmake_minimum_required(VERSION 3.0.0)
project(task)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 查找 Ceres Solver
find_package(Ceres REQUIRED)

# 查找 Eigen
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# 添加源文件
aux_source_directory(./src ALL_SRCS)

# 添加可执行文件
add_executable(task main.cpp ${ALL_SRCS})

# 链接 OpenCV 和 Ceres Solver 库
target_link_libraries(task
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)