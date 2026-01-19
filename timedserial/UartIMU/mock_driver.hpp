//
// Mock Driver - Dummy driver for testing and initialization
// Provides fixed initial data without real hardware
//

#ifndef TIMEDSERIAL_MOCK_DRIVER_H
#define TIMEDSERIAL_MOCK_DRIVER_H

// modules
#include "common.hpp"
#include "../serial_interface.hpp"

// packages
#include <stdint.h>
#include <string>
#include <thread>
#include <atomic>

/**
 * @brief 模拟串口驱动实现
 * 
 * 用途：
 * - 测试和开发时不依赖真实硬件
 * - 提供固定的初始状态数据
 * - 满足 TimedSerial 模块的状态初始化需求
 * 
 * 特性：
 * - 启动时发送一次固定的姿态和状态数据
 * - 不接收任何数据，transmit_cmd 为空操作
 * - 轻量级，无实际硬件交互
 */
class MockDriver : public SerialInterface 
{
private:
    // 保存两个独立的回调
    AttitudeCallback attitude_cb_;
    RobotStatusCallback status_cb_;
    
    // 固定的初始状态数据
    Attitude initial_attitude_;
    RobotStatus initial_status_;
    
    std::atomic<bool> initialized_{false};

public:
    /**
     * @brief 构造函数 - 使用默认初始状态
     */
    MockDriver()
    {
        // 设置默认机器人状态
        initial_status_.robot_speed_mps = 28.0f;
        initial_status_.enemy_color = EnemyColor::RED;
        initial_status_.game_state = GameState::COMMON;
        initial_status_.program_mode = ProgramMode::AUTO_AIM;
        for (int i = 0; i < 6; i++) {
            initial_status_.enemy[i] = 600;  // 满血
        }
        // initial_attitude_ 使用 Attitude 类的默认初始值（水平姿态）
    }
    
    /**
     * @brief 构造函数 - 使用自定义初始状态
     * @param att 初始姿态
     * @param status 初始机器人状态
     */
    MockDriver(const Attitude& att, const RobotStatus& status)
        : initial_attitude_(att), initial_status_(status)
    {
    }
    
    // 实现 SerialInterface 的注册接口
    void set_attitude_callback(AttitudeCallback cb) override { attitude_cb_ = cb; }
    void set_robot_status_callback(RobotStatusCallback cb) override { status_cb_ = cb; }

    // 实现生命周期管理接口
    bool init() override
    {
        LOGM_S("[MockDriver] Initializing mock driver");
        initialized_ = true;
        return true;
    }
    
    void start() override
    {
        LOGM_S("[MockDriver] Starting mock driver - sending initial data");
        
        // 发送一次初始数据
        if (attitude_cb_) {
            attitude_cb_(initial_attitude_);
            LOGM_S("[MockDriver] Sent initial attitude: yaw=%.2f, pitch=%.2f", 
                   initial_attitude_.yaw(), initial_attitude_.pitch());
        }
        
        if (status_cb_) {
            status_cb_(initial_status_);
            LOGM_S("[MockDriver] Sent initial status: enemy_color=%d, speed=%.2f", 
                   static_cast<int>(initial_status_.enemy_color), 
                   initial_status_.robot_speed_mps);
        }
    }
    
    void close() override
    {
        LOGM_S("[MockDriver] Closing mock driver");
        initialized_ = false;
    }
    
    // 实现发送接口 - 空操作
    void transmit_cmd(float yaw, float yaw_spd, float pitch, float pitch_spd, float yaw_acc, float pitch_acc, float dist, uint8_t shoot = 1, uint8_t target_id = 0) override
    {
        // Mock driver does nothing on transmit
        // Optionally log for debugging
        int shoot_flag = static_cast<int>(shoot);
        if (false) {  // Set to true for verbose debugging
            LOGM_S("[MockDriver] yaw=%.2f, yaw_spd=%.2f, pitch=%.2f,pitch_spd=%.2f, dist=%.2f, shoot=%d", yaw,yaw_spd, pitch,pitch_spd, dist, shoot_flag);
        }
    }

    // 检查是否已初始化
    bool is_initialized() const { return initialized_; }

    // 析构函数
    ~MockDriver() { close(); }
};

#endif // TIMEDSERIAL_MOCK_DRIVER_H
