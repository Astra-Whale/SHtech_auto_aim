#ifndef SENSOR_IMU_UARTIMU_H
#define SENSOR_IMU_UARTIMU_H
#include "imu.hpp"

//modules
#include "comm.hpp"
#include "common.hpp"

//packages
#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class UartIMU : public ImuHead
{
private:
    Comm *comm;
    Attitude attitude_buf;
    RobotStatus robotstatus_buf;
    bool mRun;
    std::thread t_imu_read;

public:
    UartIMU(Comm *_comm) : comm(_comm) {}
    bool init() final { return comm != nullptr; }
    void start() final
    {
        mRun = true;
        t_imu_read = std::thread(&UartIMU::read_handler, this);
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
        attitude = attitude_buf;
    }
    void get_quaternion(Eigen::Quaternionf &q) final
    {
        attitude_buf.toQuaternion(q);
    }
    void get_robotstatus(RobotStatus &robotstatus) final
    {
        robotstatus = robotstatus_buf;
    }
    ~UartIMU()
    {
        close();
    }

private:
    void read_handler();
};

#endif //SENOSR_IMU_UARTIMU_H
