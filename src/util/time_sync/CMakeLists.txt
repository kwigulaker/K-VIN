cmake_minimum_required(VERSION 3.16)

project(SensorDataSyncPublisher)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find FastDDS and FastCDR
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

# Define include paths for generated DDS code and IDL includes
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../sensors/oakdlite
    ${CMAKE_CURRENT_SOURCE_DIR}/../../sensors/mpu6050
    /usr/include/fastdds
    /usr/include/fastrtps
)
# Define your executable and all the generated .cxx files + your source
add_executable(SensorDataSyncPublisher
    SensorDataSyncPublisher.cpp
    SensorDataSync.cxx
    SensorDataSyncPubSubTypes.cxx

    ../../sensors/oakdlite/FrameMSG.cxx
    ../../sensors/oakdlite/FrameMSGPubSubTypes.cxx

    ../../sensors/mpu6050/MPU6050MSG.cxx
    ../../sensors/mpu6050/MPU6050MSGPubSubTypes.cxx
)


# Link with FastDDS
target_link_libraries(SensorDataSyncPublisher
    fastrtps
    fastcdr
)

