add_library(mpu6050
    MPU6050.cpp
)

find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

target_include_directories(mpu6050 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mpu6050
    PRIVATE nlohmann_json::nlohmann_json
    fastrtps
    fastcdr
)

add_executable(MPU6050Publisher
    MPU6050Publisher.cpp
    MPU6050MSG.cxx
    MPU6050MSGPubSubTypes.cxx
)

target_link_libraries(MPU6050Publisher
    PRIVATE mpu6050
    fastrtps
    fastcdr
)

include_directories(
    /usr/include/fastdds
    /usr/include/fastrtps
)

