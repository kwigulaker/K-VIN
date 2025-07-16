#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include "Frame.h"
#include "FramePubSubTypes.h"
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
Frame convertToDDSFrame(const cv::Mat& mat, const std::string& encoding, int64_t timestamp) {
    Frame msg;
    msg.width(mat.cols);
    msg.height(mat.rows);
    msg.step(mat.step);
    msg.encoding(encoding);
    msg.timestamp(timestamp);
    std::memcpy(msg.data().data(),mat.data,msg.data().size());
    return msg;
}

int main() {
    DomainParticipantQos qos;
    qos.name("StereoDepthFramePublisher");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0,qos);
    
    TypeSupport frameTypeDDS(new FramePubSubType());
    frameTypeDDS.register_type(participant);
    
    Topic* topicFrameL = participant->create_topic("FrameCamL", frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Topic* topicFrameR = participant->create_topic("FrameCamR", frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Topic* topicFrameDepth = participant->create_topic("FrameDepth", frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
   
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    DataWriter* writerFrameL = publisher->create_datawriter(topicFrameL, DATAWRITER_QOS_DEFAULT);
    DataWriter* writerFrameR = publisher->create_datawriter(topicFrameR, DATAWRITER_QOS_DEFAULT);
    DataWriter* writerFrameDepth = publisher->create_datawriter(topicFrameDepth, DATAWRITER_QOS_DEFAULT);
    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>(); // note that stereodepth/pointclouds from depthai are in camR frame

    auto monoLeftOut = monoLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto monoRightOut = monoRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);

    auto syncedLeftQueue = stereo->syncedLeft.createOutputQueue();
    auto syncedRightQueue = stereo->syncedRight.createOutputQueue();
    auto depthQueue = stereo->depth.createOutputQueue();

    pipeline.start();
    Frame leftFrameDDS;
    Frame rightFrameDDS;
    Frame depthFrameDDS;
    std::string encodingRectifiedImages = "mono8";
    std::string encodingDepthImages = "16UC1"; 

    auto period = std::chrono::milliseconds(200); // 5hz
    auto next = std::chrono::steady_clock::now() + period;

    while(true) {
        auto leftSynced = syncedLeftQueue->get<dai::ImgFrame>();
        auto rightSynced = syncedRightQueue->get<dai::ImgFrame>();
        auto depthSynced = depthQueue->get<dai::ImgFrame>();
        // Note we use the system clock to timestamp frames and not the oakd's internal clock
        int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
	).count();

        cv::Mat leftFrameCV = leftSynced->getCvFrame();
	cv::Mat rightFrameCV = rightSynced->getCvFrame();
        cv::Mat depthFrameCV = depthSynced->getCvFrame();

        leftFrameDDS = convertToDDSFrame(leftFrameCV,encodingRectifiedImages,ts);
        rightFrameDDS = convertToDDSFrame(rightFrameCV,encodingRectifiedImages,ts);
        depthFrameDDS = convertToDDSFrame(depthFrameCV,encodingDepthImages,ts);

        writerFrameL->write(&leftFrameDDS);
        writerFrameR->write(&rightFrameDDS);
        writerFrameDepth->write(&depthFrameDDS);
        std::this_thread::sleep_until(next);
        next += period;
    }

    pipeline.stop();
    return 0;
}
