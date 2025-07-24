#include <string>
#include <optional>
#include <deque>
#include <mutex>
#include "SensorDataSync.h"
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>

using namespace eprosima::fastdds::dds;

class SensorDataSynchronizer {
public:
    SensorDataSynchronizer(
        std::mutex* imuMutex,
        std::deque<MPU6050MSG>* imuBuffer,
        std::mutex* camLMutex,
        std::deque<FrameMSG>* camLBuffer,
        std::mutex* camRMutex,
        std::deque<FrameMSG>* camRBuffer,
        eprosima::fastdds::dds::DataWriter* writerSync);
    std::optional<MPU6050MSG> getBestIMU(const FrameMSG& frame);
    std::optional<FrameMSG> getBestFrame(const MPU6050MSG& imu, std::deque<FrameMSG>& frameBuffer);
    void trySyncAndPublish(const MPU6050MSG& imu);
    template <typename T>
    void pruneOldMessages(std::deque<T>& buffer, double maxTime, double timeNow);
    void publishSyncMessage(const MPU6050MSG& imu, const FrameMSG& camL, const FrameMSG& camR);
    void publishSyncMessageWithoutFrames(const MPU6050MSG& imu);
    static constexpr double maxSensorMSGOffsetSec = 0.04; // 40ms max time diff between imu and depth cam msgs
    static constexpr double maxBufferItemTimeSec = 1; // 1 sec max buffer time
private:
    std::mutex* mutexImu;
    std::deque<MPU6050MSG>* imuBuffer;
    std::mutex* mutexCamL;
    std::deque<FrameMSG>* camLBuffer;
    std::mutex* mutexCamR;
    std::deque<FrameMSG>* camRBuffer;
    eprosima::fastdds::dds::DataWriter* writerSync;
};
