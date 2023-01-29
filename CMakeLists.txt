cmake_minimum_required(VERSION 3.0.0)
project(NoobCalibration VERSION 0.1.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(test ./src/test.cpp)
target_link_libraries(test ${OpenCV_LIBS})
target_link_libraries(test libjsoncpp.a)

add_executable(get_pixel ./src/get_pixel.cpp)
target_link_libraries(get_pixel ${OpenCV_LIBS})

add_executable(PnP ./src/PnP.cpp)
target_link_libraries(PnP ${OpenCV_LIBS})
target_link_libraries(PnP libjsoncpp.a)

add_executable(bat3D ./src/bat3D.cpp)
target_link_libraries(bat3D ${OpenCV_LIBS})
target_link_libraries(bat3D libjsoncpp.a)

add_executable(once ./test/once/once.cpp)
target_link_libraries(once ${OpenCV_LIBS})
target_link_libraries(once libjsoncpp.a)