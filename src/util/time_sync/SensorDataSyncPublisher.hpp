class TimeSynchronizer {
public:
    void onIMU(const MPU6050MSG& imu_msg);
    void onFrame(const Frame& frame_msg);

    std::optional<SensorDataSync> trySync();

private:
    std::mutex mutex_;
    std::deque<MPU6050MSG> imuBuffer;
    std::deque<Frame> camLFrameBuffer;
    std::deque<Frame> camRFrameBuffer;
    std::deque<Frame> depthFrameBuffer;

    Frame latest_frame_;
    bool has_new_frame_ = false;

    const size_t max_buffer_size_ = 1000;
};
