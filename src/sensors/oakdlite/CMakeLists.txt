cmake_minimum_required(VERSION 3.16)
project(oakdlite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(depthai REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

add_executable(StereoDepthFramePublisher 
    StereoDepthFramePublisher.cpp
    FrameMSG.cxx
    FrameMSGPubSubTypes.cxx
)

target_include_directories(StereoDepthFramePublisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(StereoDepthFramePublisher
    depthai::core
    ${OpenCV_LIBS}
    fastrtps
    fastcdr
)

include_directories(
    /usr/include/fastdds
    /usr/include/fastrtps
)


