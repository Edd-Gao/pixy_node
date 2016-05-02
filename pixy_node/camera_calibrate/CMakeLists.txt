cmake_minimum_required (VERSION 2.8)
project (camera_calibrate CXX)

set(CMAKE_BUILD_TYPE Debug)

find_package (OpenCV REQUIRED)

add_executable (camera_calibrate camera_calibration.cpp)

target_link_libraries (camera_calibrate ${OpenCV_LIBS})
