#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include "FrameMSG.h"
#include "FrameMSGPubSubTypes.h"
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
using namespace eprosima::fastdds::rtps;

FrameMSG convertToDDSFrame(const cv::Mat& mat, const std::string& encoding, int64_t timestamp) {
    FrameMSG msg;
    msg.width(mat.cols);
    msg.height(mat.rows);
    msg.step(mat.step);
    msg.timestamp(timestamp);

    std::vector<uchar> buffer;

    if (encoding == "mono8") {
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", mat, buffer, params);
        msg.encoding("jpeg");
    }
    else {
        throw std::runtime_error("Unsupported encoding: " + encoding);
    }

    msg.data().assign(buffer.begin(), buffer.end());
    return msg;
}


int main() {
    DomainParticipantQos qos;
    qos.name("StereoDepthFramePublisher");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0,qos);

    TypeSupport frameTypeDDS(new FrameMSGPubSubType());
    frameTypeDDS.register_type(participant);
    Topic* topicFrameL = participant->create_topic("FrameCamL", frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Topic* topicFrameR = participant->create_topic("FrameCamR", frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);

    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    DataWriter* writerFrameL = publisher->create_datawriter(topicFrameL, DATAWRITER_QOS_DEFAULT);
    DataWriter* writerFrameR = publisher->create_datawriter(topicFrameR, DATAWRITER_QOS_DEFAULT);
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
    FrameMSG leftFrameDDS;
    FrameMSG rightFrameDDS;
    FrameMSG depthFrameDDS;
    std::string encodingRectifiedImages = "mono8";

    auto period = std::chrono::milliseconds(100); // 10hz
    auto next = std::chrono::steady_clock::now() + period;

    while(true) {
        // Note we use the system clock to timestamp frames and not the oakd's internal clock
        int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        auto leftSynced = syncedLeftQueue->get<dai::ImgFrame>();
        auto rightSynced = syncedRightQueue->get<dai::ImgFrame>();
        auto depthSynced = depthQueue->get<dai::ImgFrame>();

        cv::Mat leftFrameCV = leftSynced->getCvFrame();
        cv::Mat rightFrameCV = rightSynced->getCvFrame();

        leftFrameDDS = convertToDDSFrame(leftFrameCV,encodingRectifiedImages,ts);
        rightFrameDDS = convertToDDSFrame(rightFrameCV,encodingRectifiedImages,ts);
        
        writerFrameL->write(&leftFrameDDS);
        writerFrameR->write(&rightFrameDDS);
        std::this_thread::sleep_until(next);
        next += period;
    }

    pipeline.stop();
    return 0;
}