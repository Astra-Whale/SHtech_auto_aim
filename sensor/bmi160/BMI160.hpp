#ifndef SENSOR_IMU_BMI160_H
#define SENSOR_IMU_BMI160_H
#include "imu.hpp"
#include "MahonyAHRS.h"
#include "i2c/i2c.h"

//modules
#include "common.hpp"
#include "comm.hpp"

#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#define SAMPLERATE 500
#define DURATION_US 1000000 / SAMPLERATE

class BMI160 : public ImuHead
{
private:
    I2CDevice bmi_i2c;
    int bus;
    Mahony mMahony;
    bool mRun;
    bool init_success;
    std::thread t_imu_read;

public:
    BMI160() : init_success(false){};
    bool init() final;
    void start() final
    {
        mRun = true;
        t_imu_read = std::thread(&BMI160::read_handler, this);
    }
    void close() final
    {
        if (mRun)
        {
            mRun = false;
        }
        t_imu_read.join();
    }
    void get_attitude(Attitude &attitude) final
    {
        attitude.yaw = mMahony.getYaw();
        attitude.pitch = mMahony.getPitch();
        attitude.roll = mMahony.getRoll();
    }
    void get_quaternion(Eigen::Quaternionf &q) final
    {
    }
    ~BMI160()
    {
        close();
        i2c_close(bus);
    }

private:
    void read_handler();
};

#endif
