//
// Created by Haoran Jiang on 21-10-10: Add IMU support - BMI160.
// Capture image from input sources
//

#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

//modules
#include "common.hpp"

class ImuHead
{
public:
    virtual ~ImuHead() = default;
    virtual bool init() = 0;
    virtual void get_attitude(Attitude &_attitude) = 0;
    virtual void get_quaternion(Eigen::Quaternionf&) = 0;
    virtual void get_robotstatus(RobotStatus &robotstatus) = 0;
    virtual void start() = 0;
    virtual void close() = 0;
};

//submodules
#include "UartIMU/uartimu.hpp"

#endif
