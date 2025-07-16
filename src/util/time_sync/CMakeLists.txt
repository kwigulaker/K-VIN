cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

add_executable(SensorDataSyncPublisher
    SensorDataSyncPublisher.cpp
    Frame.cxx
    MPU6050MSG.cxx
)

target_include_directories(StereoDepthFramePublisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(StereoDepthFramePublisher
    fastrtps
    fastcdr
)

include_directories(
    /usr/include/fastdds
    /usr/include/fastrtps
)
