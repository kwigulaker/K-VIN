#include <string>
#include <optional>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "SensorDataSync.h"
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
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
        std::deque<StateVector>* stateBuffer
        );
    void initializeConfig();
    void onDataAvailable(SensorDataSync& syncMsg);
    cv::Mat convertFromDDSFrame(const FrameMSG& msg);
    void predict(const SensorDataSync& syncMsg);
    void update(const SensorDataSync& syncMsg);
    cv::Mat camL_to_IMU(cv::Mat& cam_tf);
    cv::Mat estimate_motion_with_frames(const SensorDataSync& syncMsg);
    std::unordered_map<int, cv::Point3f> projectPointsTo3D(
    std::vector<cv::KeyPoint>& kp_L,
    std::vector<cv::KeyPoint>& kp_R,
    std::vector<cv::DMatch>& matches);
    template <typename T>
    void pruneOldMessages(std::deque<T>& buffer, double maxTime, double timeNow);

private:
    std::mutex* sensorDataSyncMutex;
    std::deque<SensorDataSync>* sensorDataSyncBuffer;
    std::mutex* stateMutex;
    std::deque<StateVector>* stateBuffer;
    static constexpr double maxBufferItemTimeMilliSec = 1000; // 1 sec max buffer time
    // openCV feature detection and matching
    cv::Ptr<cv::ORB> orbDetector;
    cv::BFMatcher matcher;
    // cam intrinsics
    static constexpr const char* cameraIntrinsicsFile = "./src/odometry/cameraIntrinsics.json";
    cv::Mat camLIntrinsics;
    cv::Mat camRIntrinsics;
    // cam extrinsics
    static constexpr const char* TCamLCamRFile = "./src/odometry/camL_to_camR_m.json";
    static constexpr const char* TCamRIMUFile = "./src/odometry/camR_to_IMU_m.json";
    cv::Mat T_camL_to_camR_m;
    cv::Mat T_camR_to_IMU_m;
};
