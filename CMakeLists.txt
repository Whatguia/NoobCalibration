cmake_minimum_required(VERSION 3.0.0)
project(NoobCalibration VERSION 0.1.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(get_pixel ./src/get_pixel.cpp)
target_link_libraries(get_pixel ${OpenCV_LIBS})
target_link_libraries(get_pixel libjsoncpp.a)

add_executable(PnP ./src/PnP.cpp)
target_link_libraries(PnP ${OpenCV_LIBS})
target_link_libraries(PnP libjsoncpp.a)

add_executable(projection ./src/projection.cpp)
target_link_libraries(projection ${OpenCV_LIBS})
target_link_libraries(projection libjsoncpp.a)