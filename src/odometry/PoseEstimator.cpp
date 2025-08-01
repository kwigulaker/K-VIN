#include <iostream>
#include <chrono>
#include "PoseEstimator.hpp"
#include "MPU6050MSGPubSubTypes.h"
#include "FrameMSGPubSubTypes.h"
#include "SensorDataSync.h"
#include "SensorDataSyncPubSubTypes.h"
#include "SensorDataSyncPublisher.hpp"
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

PoseEstimator::PoseEstimator(
    std::mutex* sensorDataSyncMutex,
    std::deque<SensorDataSync>* sensorDataSyncBuffer,
    std::mutex* stateMutex,
    std::deque<StateVector>* stateBuffer
) :
    sensorDataSyncMutex(sensorDataSyncMutex),
    sensorDataSyncBuffer(sensorDataSyncBuffer),
    stateMutex(stateMutex),
    stateBuffer(stateBuffer)
{
    initializeConfig();
    // Initialize ORB detector and matcher
    orbDetector = cv::ORB::create();
    matcher = cv::BFMatcher(cv::NORM_HAMMING, true); // Use Hamming distance for ORB
    std::cout << "[PoseEstimator] Initialized with ORB detector and BFMatcher." << std::endl;
}



class SensorDataSyncListener : public DataReaderListener {
public:
    PoseEstimator* estimator_;

    SensorDataSyncListener(PoseEstimator* estimator)
        : estimator_(estimator) {}

    void on_data_available(DataReader* reader) override {
        SensorDataSync msg;
        SampleInfo info;
        if (reader->take_next_sample(&msg, &info) == 0 && info.valid_data) {
            estimator_->onDataAvailable(msg);
        }
    }
};

void PoseEstimator::initializeConfig() {
    // Load camera intrinsics
    {
        std::ifstream file(cameraIntrinsicsFile);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open cameraIntrinsicsFile");
        }

        nlohmann::json j;
        file >> j;

        auto toMat3x3 = [](const nlohmann::json& m) {
            cv::Mat mat(3, 3, CV_64F);
            for (int i = 0; i < 3; ++i)
                for (int jx = 0; jx < 3; ++jx)
                    mat.at<double>(i, jx) = m[i][jx];
            return mat;
        };

        camLIntrinsics = toMat3x3(j["CamL"]["intrinsic_matrix"]);
        camRIntrinsics = toMat3x3(j["CamR"]["intrinsic_matrix"]);
    }

    // Load T_camL_to_camR
    {
        std::ifstream file(TCamLCamRFile);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open TCamLCamRFile");
        }

        nlohmann::json j;
        file >> j;

        T_camL_to_camR_m = cv::Mat(4, 4, CV_64F);
        for (int i = 0; i < 4; ++i)
            for (int jx = 0; jx < 4; ++jx)
                T_camL_to_camR_m.at<double>(i, jx) = j["T_camL_to_camR"][i][jx];
    }

    // Load T_camR_to_IMU
    {
        std::ifstream file(TCamRIMUFile);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open TCamRIMUFile");
        }

        nlohmann::json j;
        file >> j;

        T_camR_to_IMU_m = cv::Mat(4, 4, CV_64F);
        for (int i = 0; i < 4; ++i)
            for (int jx = 0; jx < 4; ++jx)
                T_camR_to_IMU_m.at<double>(i, jx) = j["T_camR_to_IMU"][i][jx];
    }

    std::cout << "[PoseEstimator] Loaded camera intrinsics and extrinsics." << std::endl;
}

void PoseEstimator::onDataAvailable(SensorDataSync& syncMsg) {
    if(syncMsg.cam_L().timestamp() == -1) {
        predict(syncMsg);
    } else {
        update(syncMsg);
    }
    std::lock_guard<std::mutex> lock(*sensorDataSyncMutex);
    sensorDataSyncBuffer->push_back(syncMsg);
    pruneOldMessages<SensorDataSync>(
        *sensorDataSyncBuffer,
        maxBufferItemTimeMilliSec,
        syncMsg.timestamp()
    );
}

template <typename T>
void PoseEstimator::pruneOldMessages(std::deque<T>& buffer, double maxTime, double timeNow){
    while (!buffer.empty() && (timeNow - buffer.front().timestamp()) > maxTime) {
        buffer.pop_front();
    }
};

void PoseEstimator::predict(const SensorDataSync& syncMsg) {
    // Placeholder for prediction logic
    //std::cout << "[PoseEstimator] Predicting state based on IMU data @ timestamp: "
    //          << syncMsg.imu_parsed().timestamp() << std::endl;
    // Here you would implement the prediction step using the IMU data
}

void PoseEstimator::update(const SensorDataSync& syncMsg) {
    cv::Mat tf_camL_t1_t = estimate_motion_with_frames(syncMsg); // Matrix T is tf between t-1 and t in the cam L frame
    if (tf_camL_t1_t.empty()) {
        return;
    }
    cv::Mat tf_IMU_t1_t = camL_to_IMU(tf_camL_t1_t); // perform frame change to IMU frame
    std::cout << "[PoseEstimator] Estimated T_t1_to_t:\n" << tf_IMU_t1_t << std::endl;
}

cv::Mat PoseEstimator::camL_to_IMU(cv::Mat& cam_L_t1_t){
    cv::Mat T_camL_to_IMU = T_camR_to_IMU_m * T_camL_to_camR_m;
    cv::Mat T_IMU_to_camL = T_camL_to_IMU.inv();
    cv::Mat T_t1_to_t_IMU = T_camL_to_IMU * cam_L_t1_t * T_IMU_to_camL;
    return T_t1_to_t_IMU;
}

cv::Mat PoseEstimator::estimate_motion_with_frames(const SensorDataSync& syncMsg) {
    // Check if there's any previous sensor data to use
    if (sensorDataSyncBuffer->empty()) {
        return cv::Mat();
    }
    // Check if previous sensor data has frames from depth camera
    SensorDataSync* prevSyncMsg = nullptr;
    for (auto it = sensorDataSyncBuffer->rbegin(); it != sensorDataSyncBuffer->rend(); ++it) {
    if (it->cam_L().timestamp() != -1) {
        prevSyncMsg = &(*it);
        break;
        }
    }
    if(!prevSyncMsg){
        return cv::Mat();
    }
    // We have frames in t-1 and t, so we can perform odometry
    cv::Mat frameL = convertFromDDSFrame(syncMsg.cam_L());
    cv::Mat frameR = convertFromDDSFrame(syncMsg.cam_R());
    std::vector<cv::KeyPoint> kp_L, kp_R;
    cv::Mat desc_L, desc_R;
    orbDetector->detectAndCompute(frameL, cv::noArray(), kp_L, desc_L);
    orbDetector->detectAndCompute(frameR, cv::noArray(), kp_R, desc_R);
    // Match feature points between left and right frames
    std::vector<cv::DMatch> matches_L_R_t_1;
    std::vector<cv::DMatch> matches_L_R_t;
    std::vector<cv::DMatch> matches_L_t_1_to_t;
    matcher.match(desc_L, desc_R, matches_L_R_t);
    // Also match feature points between previous left frame and right frames and left frames between t-1 and t
    cv::Mat prevFrameL = convertFromDDSFrame(prevSyncMsg->cam_L());
    cv::Mat prevFrameR = convertFromDDSFrame(prevSyncMsg->cam_R());
    std::vector<cv::KeyPoint> prevKp_L;
    std::vector<cv::KeyPoint> prevKp_R;
    cv::Mat prevDesc_L;
    cv::Mat prevDesc_R;
    orbDetector->detectAndCompute(prevFrameL, cv::noArray(), prevKp_L, prevDesc_L);
    orbDetector->detectAndCompute(prevFrameR, cv::noArray(), prevKp_R, prevDesc_R);
    matcher.match(desc_L, prevDesc_L, matches_L_t_1_to_t);
    matcher.match(prevDesc_L, prevDesc_R, matches_L_R_t_1);
    // Project matched keypoints in t-1 to 3D in the Cam L frame.
    auto pointMap3D = projectPointsTo3D(prevKp_L, prevKp_R, matches_L_R_t_1);
    std::vector<cv::Point3f> points3D_t1;  // world points from t-1
    std::vector<cv::Point2f> points2D_t;  // image points in current frame (t)

    for (const auto& match : matches_L_t_1_to_t) {
    int idx_t1 = match.trainIdx;
    int idx_t = match.queryIdx;

    auto it = pointMap3D.find(idx_t1);
    if (it != pointMap3D.end()) {
            points3D_t1.push_back(it->second);
            points2D_t.push_back(kp_L[idx_t].pt);
        }
    }

    cv::Mat rvec, tvec;
    bool success = cv::solvePnPRansac(
        points3D_t1,        // 3D points from t-1
        points2D_t,         // matched 2D points in t
        camLIntrinsics,     // camera matrix
        cv::noArray(),      // assume undistorted or already rectified
        rvec,
        tvec,
        false,
        100,                // iterations
        8.0,                // reprojection error in px
        0.99                // confidence
    );
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
}

cv::Mat PoseEstimator::convertFromDDSFrame(const FrameMSG& msg) {
    std::vector<uchar> buffer(msg.data().begin(), msg.data().end());
    cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
    if (decoded.empty()) {
        throw std::runtime_error("Failed to decode FrameMSG data with encoding: " + msg.encoding());
    }
    if (decoded.cols != msg.width() || decoded.rows != msg.height()) {
        std::cerr << "[Warning] Decoded image size mismatch: got "
                  << decoded.cols << "x" << decoded.rows
                  << ", expected " << msg.width() << "x" << msg.height() << std::endl;
    }
    return decoded;
}

std::unordered_map<int, cv::Point3f> PoseEstimator::projectPointsTo3D(
    std::vector<cv::KeyPoint>& kp_L,
    std::vector<cv::KeyPoint>& kp_R,
    std::vector<cv::DMatch>& matches)
{
    std::unordered_map<int, cv::Point3f> pointMap;

    double fx = camLIntrinsics.at<double>(0, 0);
    double cx = camLIntrinsics.at<double>(0, 2);
    double fy = camLIntrinsics.at<double>(1, 1);
    double cy = camLIntrinsics.at<double>(1, 2);
    double baseline = std::abs(T_camL_to_camR_m.at<double>(0, 3));

    for (const auto& m : matches) {
        const cv::Point2f& ptL = kp_L[m.queryIdx].pt;
        const cv::Point2f& ptR = kp_R[m.trainIdx].pt;

        float disparity = ptL.x - ptR.x;
        if (disparity <= 0.0f) continue;

        float Z = static_cast<float>((fx * baseline) / disparity);
        float X = static_cast<float>((ptL.x - cx) * Z / fx);
        float Y = static_cast<float>((ptL.y - cy) * Z / fy);

        pointMap[m.queryIdx] = cv::Point3f(X, Y, Z);
    }

    return pointMap;
}


int main() {
    // Topic
    constexpr const char* syncTopicName = "SensorDataSync";
    DomainParticipantQos qos;
    qos.name("PoseEstimator");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, qos);

    // Buffers
    std::deque<SensorDataSync> sensorDataSyncBuffer;
    std::mutex mutexSensorDataSync;

    std::deque<StateVector> stateBuffer;
    std::mutex mutexState;

    // Listeners
    PoseEstimator* estimator = new PoseEstimator(&mutexSensorDataSync,&sensorDataSyncBuffer,&mutexState,&stateBuffer);
    SensorDataSyncListener* listener = new SensorDataSyncListener(estimator);

    // data subscriber init
    TypeSupport sensorDataSyncTypeDDS(new SensorDataSyncPubSubType());
    sensorDataSyncTypeDDS.register_type(participant);
    Topic* topicSensorDataSync = participant->create_topic(syncTopicName, sensorDataSyncTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Subscriber* subscriberSensorDataSync = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    DataReader* readerSensorDataSync = subscriberSensorDataSync->create_datareader(topicSensorDataSync,DATAREADER_QOS_DEFAULT,listener);

    std::cout << "Running Pose Estimator... Ctrl+C to exit." << std::endl;
    std::this_thread::sleep_for(std::chrono::hours(24));
    return 0;
}
