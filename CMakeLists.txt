set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.10)

project(OpenCVTracker)

find_package(OpenCV REQUIRED)
add_executable(OpenCVTracker main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(OpenCVTracker ${OpenCV_LIBS})

