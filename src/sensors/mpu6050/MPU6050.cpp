#include "MPU6050.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

using json = nlohmann::json;

MPU6050::MPU6050(const std::string& i2c_device, const std::string& calibration_file)
    : device_(i2c_device), calib_file_(calibration_file) {}

bool MPU6050::initialize() {
    std::cout << "[MPU6050] Opening device: " << device_ << std::endl;

    i2c_fd_ = open(device_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        std::cerr << "[MPU6050] Failed to open I2C device: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "[MPU6050] Setting I2C address to 0x" << std::hex << MPU6050_ADDR << std::endl;
    if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "[MPU6050] Failed to set I2C address: " << strerror(errno) << std::endl;
        return false;
    }

    uint8_t buf[2] = { PWR_MGMT_1, 0 };
    if (write(i2c_fd_, buf, 2) != 2) {
        std::cerr << "[MPU6050] Failed to wake MPU6050: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "[MPU6050] Initialization successful." << std::endl;
    return loadCalibration();
}


bool MPU6050::loadCalibration() {
    std::ifstream file(calib_file_);
    if (!file.is_open()) return false;

    json calib;
    file >> calib;

    accel_bias_ = {
        calib["accel_bias"]["x"],
        calib["accel_bias"]["y"],
        calib["accel_bias"]["z"]
    };

    gyro_bias_ = {
        calib["gyro_bias"]["x"],
        calib["gyro_bias"]["y"],
        calib["gyro_bias"]["z"]
    };

    return true;
}

bool MPU6050::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                       int16_t& gx, int16_t& gy, int16_t& gz) {
    uint8_t buf[14];
    buf[0] = ACCEL_XOUT_H;

    if (write(i2c_fd_, buf, 1) != 1) return false;
    if (read(i2c_fd_, buf, 14) != 14) return false;

    ax = (buf[0] << 8) | buf[1];
    ay = (buf[2] << 8) | buf[3];
    az = (buf[4] << 8) | buf[5];

    gx = (buf[8] << 8) | buf[9];
    gy = (buf[10] << 8) | buf[11];
    gz = (buf[12] << 8) | buf[13];

    return true;
}

IMU MPU6050::readProcessed() {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(ax, ay, az, gx, gy, gz)) {
        std::cerr << "Failed to read IMU\n";
        return {};
    }

    constexpr float accel_scale = 16384.0;   // raw -> g
    constexpr float gyro_scale = 131.0;      // raw -> deg/s
    constexpr float g = 9.80665;
    constexpr float deg2rad = M_PI / 180.0;

    return {
        {
            (ax / accel_scale - accel_bias_[0]) * g,
            (ay / accel_scale - accel_bias_[1]) * g,
            (az / accel_scale - accel_bias_[2]) * g
        },
        {
            (gx / gyro_scale - gyro_bias_[0]) * deg2rad,
            (gy / gyro_scale - gyro_bias_[1]) * deg2rad,
            (gz / gyro_scale - gyro_bias_[2]) * deg2rad
        },
        std::chrono::system_clock::now(),
    };
}
