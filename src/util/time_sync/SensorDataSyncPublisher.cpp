#include <chrono>
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
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

SensorDataSynchronizer::SensorDataSynchronizer(
    std::mutex* imuMutex,
    std::deque<MPU6050MSG>* imuBuffer,
    std::mutex* camLMutex,
    std::deque<FrameMSG>* camLBuffer,
    std::mutex* camRMutex,
    std::deque<FrameMSG>* camRBuffer,
    eprosima::fastdds::dds::DataWriter* writerSync
) :
    mutexImu(imuMutex),
    imuBuffer(imuBuffer),
    mutexCamL(camLMutex),
    camLBuffer(camLBuffer),
    mutexCamR(camRMutex),
    camRBuffer(camRBuffer),
    writerSync(writerSync)
{}

std::optional<FrameMSG> SensorDataSynchronizer::getBestFrame(const MPU6050MSG& imu, std::deque<FrameMSG>& frameBuffer) {
    if (frameBuffer.empty()) {
        return std::nullopt;
    }

    double targetTime = imu.timestamp();
    const FrameMSG* best = nullptr;
    double bestDiff = std::numeric_limits<double>::max();

    for (const auto& frame : frameBuffer) {
        double diff = std::abs(frame.timestamp() - targetTime);
        if (diff < bestDiff) {
            best = &frame;
            bestDiff = diff;
        }
    }
    double maxSensorOffsetMSGMilliSec = SensorDataSynchronizer::maxSensorMSGOffsetSec * 1000.0; // convert to milliseconds
    if (bestDiff <= maxSensorOffsetMSGMilliSec) {
        return *best;
    } else {
        return std::nullopt;
    }
}

void SensorDataSynchronizer::trySyncAndPublish(const MPU6050MSG& imu) {
    FrameMSG camL, camR;
    bool haveAll = true;
    double timeNow = imu.timestamp();
    {
        std::lock_guard<std::mutex> lock(*mutexCamL);
        auto opt = getBestFrame(imu, *camLBuffer);
        if (opt) camL = *opt;
        else haveAll = false;
        pruneOldMessages(*camLBuffer,maxBufferItemTimeSec,timeNow);
    }
    {
        std::lock_guard<std::mutex> lock(*mutexCamR);
        auto opt = getBestFrame(imu, *camRBuffer);
        if (opt) camR = *opt;
        else haveAll = false;
        pruneOldMessages(*camRBuffer,maxBufferItemTimeSec,timeNow);
    }
    if (haveAll) {
        publishSyncMessage(imu, camL, camR);
    }
    else {
        publishSyncMessageWithoutFrames(imu);
    }
};

template <typename T>
void SensorDataSynchronizer::pruneOldMessages(std::deque<T>& buffer, double maxTime, double timeNow){
    while (!buffer.empty() && (timeNow - buffer.front().timestamp()) > maxTime) {
        buffer.pop_front();
    }
};

void SensorDataSynchronizer::publishSyncMessage(const MPU6050MSG& imu, const FrameMSG& camL, const FrameMSG& camR){
    SensorDataSync syncMsg;
    syncMsg.cam_L(camL);
    syncMsg.cam_R(camR);
    syncMsg.imu_parsed(imu);
    syncMsg.timestamp(imu.timestamp());
    writerSync->write(&syncMsg);
    //std::cout << "[Sync] Published full message @ " << imu.timestamp() << " with synced frames." << std::endl;
};

void SensorDataSynchronizer::publishSyncMessageWithoutFrames(const MPU6050MSG& imu){
    FrameMSG dummy;
    dummy.timestamp(-1.0);  // use this to check if frames not available in ekf
    SensorDataSync syncMsg;
    syncMsg.cam_L(dummy);
    syncMsg.cam_R(dummy);
    syncMsg.imu_parsed(imu);
    syncMsg.timestamp(imu.timestamp());
    writerSync->write(&syncMsg);
    //std::cout << "[Sync] Published message @ " << imu.timestamp() << " without frames." << std::endl;
};

class IMUListener : public DataReaderListener {
public:
    std::mutex& mutex_;
    std::deque<MPU6050MSG>& buffer_;
    SensorDataSynchronizer& sync;

    IMUListener(std::mutex& m, std::deque<MPU6050MSG>& b, SensorDataSynchronizer& s)
        : mutex_(m), buffer_(b), sync(s) {}

    void on_data_available(DataReader* reader) override {
        MPU6050MSG msg;
        SampleInfo info;

        if (reader->take_next_sample(&msg, &info) == 0 && info.valid_data) {
            std::lock_guard<std::mutex> lock(mutex_);
            buffer_.push_back(msg);

            sync.trySyncAndPublish(msg);

            sync.pruneOldMessages<MPU6050MSG>(
                buffer_,
                SensorDataSynchronizer::maxBufferItemTimeSec,
                msg.timestamp()
            );
        }
    }
};


class FrameListener : public DataReaderListener {
public:
    std::mutex& mutex_;
    std::deque<FrameMSG>& buffer_;

    FrameListener(std::mutex& m, std::deque<FrameMSG>& b) : mutex_(m), buffer_(b) {}

    void on_data_available(DataReader* reader) override {
        FrameMSG msg;
        SampleInfo info;
        if (reader->take_next_sample(&msg, &info) == 0 && info.valid_data) {
            std::lock_guard<std::mutex> lock(mutex_);
            buffer_.push_back(msg);
        }
    }
};



int main() {
    // Topic names
    constexpr const char* imuTopicName      = "MPU6050IMU";
    constexpr const char* camLTopicName     = "FrameCamL";
    constexpr const char* camRTopicName     = "FrameCamR";
    constexpr const char* syncTopicName     = "SensorDataSync";

    // init for data buffers and listeners
    std::deque<MPU6050MSG> imuBuffer;
    std::mutex mutexImu;

    std::deque<FrameMSG> camLBuffer;
    std::mutex mutexCamL;

    std::deque<FrameMSG> camRBuffer;
    std::mutex mutexCamR;

    DomainParticipantQos qos;
    qos.name("SensorDataSyncPublisher");
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, qos);

    TypeSupport syncTypeDDS(new SensorDataSyncPubSubType());
    syncTypeDDS.register_type(participant);

    Topic* topicSync = participant->create_topic(syncTopicName, syncTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Publisher* publisherSync = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    DataWriter* writerSync = publisherSync->create_datawriter(topicSync, DATAWRITER_QOS_DEFAULT);

    // Sync class to publish on new msg
    SensorDataSynchronizer synchronizer(
        &mutexImu,&imuBuffer,&mutexCamL,&camLBuffer,&mutexCamR,&camRBuffer,writerSync
    );

    // Listeners
    IMUListener* imuListener = new IMUListener(mutexImu,imuBuffer,synchronizer);
    FrameListener* camLListener = new FrameListener(mutexCamL,camLBuffer);
    FrameListener* camRListener = new FrameListener(mutexCamR,camRBuffer);

    // IMU data subscriber init
    TypeSupport imuTypeDDS(new MPU6050MSGPubSubType());
    imuTypeDDS.register_type(participant);
    Topic* topicImu = participant->create_topic(imuTopicName, imuTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Subscriber* subscriberImu = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    DataReader* readerIMU = subscriberImu->create_datareader(topicImu,DATAREADER_QOS_DEFAULT,imuListener);
   
    // Cam L,R,Depth frame subscriber init
    TypeSupport frameTypeDDS(new FrameMSGPubSubType());
    frameTypeDDS.register_type(participant);
    Topic* topicCamL = participant->create_topic(camLTopicName, frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Subscriber* subscriberCamL = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    DataReader* readerCamL = subscriberCamL->create_datareader(topicCamL,DATAREADER_QOS_DEFAULT,camLListener);
    Topic* topicCamR = participant->create_topic(camRTopicName, frameTypeDDS.get_type_name(), TOPIC_QOS_DEFAULT);
    Subscriber* subscriberCamR = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    DataReader* readerCamR = subscriberCamR->create_datareader(topicCamR,DATAREADER_QOS_DEFAULT,camRListener);

    std::cout << "Running Synchronizer... Ctrl+C to exit." << std::endl;
    std::this_thread::sleep_for(std::chrono::hours(24));
    return 0;
};
