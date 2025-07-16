#include <chrono>
#include <array>
#include <cstdint>
#include "MPU6050.hpp"
#include "MPU6050MSG.h"
#include "MPU6050MSGPubSubTypes.h"
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

using namespace eprosima::fastdds::dds;

MPU6050MSG convertToDDSMSG(const IMU& imu_parsed) {
    MPU6050MSG msg;
    msg.ax(static_cast<int16_t>(imu_parsed.accel[0]));
    msg.ay(static_cast<int16_t>(imu_parsed.accel[1]));
    msg.az(static_cast<int16_t>(imu_parsed.accel[2]));

    msg.gx(static_cast<int16_t>(imu_parsed.gyro[0]));
    msg.gy(static_cast<int16_t>(imu_parsed.gyro[1]));
    msg.gz(static_cast<int16_t>(imu_parsed.gyro[2]));

    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        imu_parsed.timestamp.time_since_epoch()
    ).count();

    msg.timestamp(static_cast<int64_t>(ts));

    return msg;
}

int main() {
    DomainParticipantQos qos;
    qos.name("MPU6050Publisher");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0,qos);

    TypeSupport IMUTypeDDS(new MPU6050MSGPubSubType());
    IMUTypeDDS.register_type(participant);

    Topic* topicIMU = participant->create_topic("MPU6050IMU", IMUTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    DataWriter* writerIMU = publisher->create_datawriter(topicIMU, DATAWRITER_QOS_DEFAULT);
    
    std::string i2c_device = "/dev/i2c-1";
    std::string calib_file = "/home/kiwi/scout/src/sensors/mpu6050/calibration.json";

    MPU6050 mpu6050(i2c_device,calib_file);
    mpu6050.initialize();

    MPU6050MSG MPU6050IMUDDS;
    IMU imuParsed;
    auto period = std::chrono::milliseconds(2); // 500hz
    auto next = std::chrono::steady_clock::now() + period;

    while(true) {
        imuParsed = mpu6050.readProcessed();
        MPU6050IMUDDS = convertToDDSMSG(imuParsed);
        writerIMU->write(&MPU6050IMUDDS);
    }
    return 0;
}
