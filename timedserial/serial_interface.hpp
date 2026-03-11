//
// Serial Interface - Event-Driven Architecture
// Created based on refactoring guide for decoupling driver and business logic
//

#ifndef SENSOR_SERIAL_INTERFACE_H
#define SENSOR_SERIAL_INTERFACE_H

#include "common.hpp"
#include <functional>

// 1. 定义独立的回调函数类型
using AttitudeCallback = std::function<void(const Attitude&)>;
using RobotStatusCallback = std::function<void(const RobotStatus&)>;

/**
 * @brief 串口接口抽象类 - 驱动层与业务层解耦的核心接口
 * 
 * 采用事件驱动模型，通过独立的回调机制实现：
 * - 姿态数据（Attitude）到达时立即触发 AttitudeCallback
 * - 机器人状态（RobotStatus）到达时立即触发 RobotStatusCallback
 * 
 * 驱动层实现此接口，保持无状态化；业务层通过依赖注入使用此接口
 */
class SerialInterface
{
public:
    virtual ~SerialInterface() = default;

    // 生命周期管理
    virtual bool init() = 0;
    virtual void start() = 0;
    virtual void close() = 0;

    // 2. 注册回调接口 (独立注册)
    /**
     * @brief 注册姿态数据回调
     * @param cb 当接收到姿态数据时触发的回调函数
     */
    virtual void set_attitude_callback(AttitudeCallback cb) = 0;

    /**
     * @brief 注册机器人状态回调
     * @param cb 当接收到机器人状态数据时触发的回调函数
     */
    virtual void set_robot_status_callback(RobotStatusCallback cb) = 0;

    // 3. 发送接口
    /**
     * @brief 发送控制指令到下位机
     * @param yaw 目标yaw角度
     * @param yaw_spd yaw角速度
     * @param pitch 目标pitch角度
     * @param pitch_spd pitch角速度
     * @param dist 目标距离
     * @param shoot 射击指令 (默认值1表示射击)
     */
    virtual void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, float yaw_acc, float pitch_acc, float dist, uint8_t shoot = 0, uint8_t target_id = 0) = 0;
};

#endif // SENSOR_SERIAL_INTERFACE_H
