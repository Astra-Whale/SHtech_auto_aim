//
// UART Driver - Stateless driver implementation
// Created based on refactoring guide for event-driven architecture
//

#ifndef TIMEDSERIAL_UART_DRIVER_H
#define TIMEDSERIAL_UART_DRIVER_H

// modules
#include "common.hpp"
#include "../serial_interface.hpp"

// packages
#include <RMCVSerial/RMCVSerial.hpp>
#include <stdint.h>
#include <string>

/**
 * @brief 无状态UART驱动实现
 * 
 * 特性：
 * - 无状态：不保存 Attitude 和 RobotStatus 数据
 * - 事件驱动：收到数据后立即通过回调通知业务层
 * - 独立回调：姿态数据和机器人状态使用独立的回调函数
 * - 解耦：驱动层只负责数据收发和协议解析
 */
class UartDriver : public SerialInterface 
{
private:
    // 保存两个独立的回调
    AttitudeCallback attitude_cb_;
    RobotStatusCallback status_cb_;
    
    drivers::RMCVSerial m_serial;
    const std::string m_device_name;

public:
    /**
     * @brief 构造函数
     * @param device_name 串口设备名称 (如 "/dev/ttyUSB0")
     */
    UartDriver(const std::string device_name);
    
    // 实现 SerialInterface 的注册接口
    void set_attitude_callback(AttitudeCallback cb) override { attitude_cb_ = cb; }
    void set_robot_status_callback(RobotStatusCallback cb) override { status_cb_ = cb; }

    // 实现生命周期管理接口
    bool init() override;
    void start() override;
    void close() override;
    
    // 实现发送接口
    void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, float yaw_acc, float pitch_acc, float dist, uint8_t shoot = 0, uint8_t target_id = 0);

    // 检查串口是否打开
    bool is_open() { return m_serial.is_open(); }

    // 析构函数
    ~UartDriver() { close(); }

private:
    /**
     * @brief 处理IMU姿态数据包
     * @param packet_ptr 数据包指针
     * @param len 数据包长度
     * 
     * 接收到姿态数据后立即构造 Attitude 对象并触发回调
     */
    void on_receive_imu(drivers::packet_data_t *packet_ptr, drivers::packet_length_t len);
    
    /**
     * @brief 处理机器人状态数据包（裁判系统数据）
     * @param packet_ptr 数据包指针
     * @param len 数据包长度
     * 
     * 接收到裁判系统数据后立即构造 RobotStatus 对象并触发回调
     */
    void on_receive_sts(drivers::packet_data_t *packet_ptr, drivers::packet_length_t len);
};

#endif // TIMEDSERIAL_UART_DRIVER_H
