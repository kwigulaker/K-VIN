#pragma once
#include <chrono>
#include <array>
#include <string>

struct IMU {
    std::array<float, 3> accel;  // ax, ay, az
    std::array<float, 3> gyro;   // gx, gy, gz
    std::chrono::system_clock::time_point timestamp;
};

class MPU6050 {
public:
    MPU6050(const std::string& i2c_device, const std::string& calibration_file);
    bool initialize();
    IMU readProcessed();
    

private:
    int i2c_fd_;
    std::string device_;
    std::string calib_file_;

    std::array<float, 3> accel_bias_;
    std::array<float, 3> gyro_bias_;

    bool readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                 int16_t& gx, int16_t& gy, int16_t& gz);
    bool loadCalibration();
};
