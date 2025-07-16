cmake_minimum_required(VERSION 3.16)
project(scout LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Global dependencies
find_package(OpenCV REQUIRED)
find_package(depthai REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

# Add subdirectories
add_subdirectory(src/sensors/mpu6050)
add_subdirectory(src/sensors/oakdlite)

