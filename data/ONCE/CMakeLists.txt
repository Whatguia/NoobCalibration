cmake_minimum_required(VERSION 3.0.0)
project(NoobCalibration VERSION 0.1.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(../../include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(once_test ./src/once_test.cpp)
target_link_libraries(once_test ${OpenCV_LIBS})
target_link_libraries(once_test libjsoncpp.a)