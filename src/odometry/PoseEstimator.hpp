#include <string>
#include <optional>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "SensorDataSync.h"
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace eprosima::fastdds::dds;

struct StateVector {
    Eigen::Quaterniond q_world_imu;  // Attitude
    Eigen::Vector3d p_world_imu;     // Position
    Eigen::Vector3d v_world_imu;     // Linear velocity

    Eigen::Vector3d gyro_bias; // IMU bias gyroscope
    Eigen::Vector3d accel_bias; // IMU bias accelerometer

    Eigen::Matrix<double, 15, 15> P;  // Covariance

    int64_t timestamp;
};


class PoseEstimator {
public:
    PoseEstimator(
        std::mutex* sensorDataSyncMutex,
        std::deque<SensorDataSync>* sensorDataSyncBuffer,
        std::mutex* stateMutex,
        std::deque<StateVector>* stateBuffer,
        eprosima::fastdds::dds::DataWriter* writerSync);
    
    void predict(const SensorDataSync& syncMsg);
    void update(const SensorDataSync& syncMsg);
    void detectFeaturePoints(const FrameMSG& frame);
    void matchFeaturePoints(const FrameMSG& a, const FrameMSG& b);
    void projectPointsTo3D(const FrameMSG& frame);
    template <typename T>
    void pruneOldMessages(std::deque<T>& buffer, double maxTime, double timeNow);

private:
    std::mutex* sensorDataSyncMutex;
    std::deque<StateVector>* sensorDataSyncBuffer;
    std::mutex* stateMutex;
    std::deque<FrameMSG>* stateBuffer;
    eprosima::fastdds::dds::DataWriter* writerSync;
    // openCV feature detection and matching
    cv::Ptr<cv::ORB> orbDetector;
    cv::BFMatcher matcher;
};
