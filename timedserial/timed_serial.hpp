//
// Created for hardware communication module - TimedSerial (Refactored)
// Event-driven architecture with dependency injection
//

#ifndef TIMEDSERIAL_NEW_H
#define TIMEDSERIAL_NEW_H

// submodules - 使用新的接口
#include "serial_interface.hpp"

// modules
#include "common.hpp"

// packages
#include <stdint.h>
#include <string>
#include <functional>
#include <chrono>
#include <memory>

namespace hardware
{
    struct TimedSerialConfig : pipeline::ModuleConfig
    {
    };

    /**
     * @brief   串口定时通讯子模块（重构版）
     * @details 
     * 重构要点：
     * - 依赖注入：通过构造函数接收 SerialInterface 独占指针
     * - 事件驱动：通过独立回调接收姿态和状态数据
     * - 数据汇聚：作为数据的汇聚点，统一管理和合并来自驱动层的数据
     * - 线程安全：使用互斥锁保护共享数据
     */
    class TimedSerial : public pipeline::BasicTask
    {
    public:

        /**
         * @brief   构造函数（重构版 - 依赖注入）
         * @param[in] driver_impl 串口驱动实现的独占指针（依赖注入）
         * @param[in] message_bridge 消息桥接对象引用
         */
        TimedSerial(const TimedSerialConfig& config,
                        std::unique_ptr<SerialInterface> driver_impl, 
                        pipeline::bridge::PlannerToSerialBridge &planner_bridge,
                        pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge,
                        pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge
                    );
        
        virtual ~TimedSerial();

        /**
         * @brief   子模块处理函数
         */
        void operator()() override;

    private:
        TimedSerialConfig config_;
        /**
         * @brief   读取最新命令和姿态数据，基于时间戳进行线性插值
         * @return  bool 成功返回true，命令数组耗尽返回false
         */
        bool read_latest_command_and_attitude();
        
        /**
         * @brief   处理来自 Planner 的命令消息（回调函数）
         * @param[in] msg 包含命令数组和姿态的消息
         */
        void handle_planner_message(const pipeline::bridge::PlannerToSerialMessage &msg);

        /**
         * @brief   处理姿态数据更新（驱动层回调）
         * @param[in] att 最新的姿态数据
         */
        void handle_attitude_update(const Attitude& att);

        /**
         * @brief   处理机器人状态更新（驱动层回调）
         * @param[in] sts 最新的机器人状态数据
         */
        void handle_status_update(const RobotStatus& sts);

        Attitude handle_attitude_get();

        RobotStatus handle_robotstatus_get();

        // 常量定义
        static constexpr size_t CMDARRAYLENGTH = 10;
        static constexpr std::chrono::microseconds send_period{5000};
        using command_array_t = std::array<RobotCommand, CMDARRAYLENGTH>;
        
        // 通讯相关成员变量
        std::unique_ptr<SerialInterface> driver_; /*!< 串口驱动独占指针（依赖注入） */
        pipeline::bridge::PlannerToSerialBridge &planner_bridge_; /*!< 消息桥接引用 */
        pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge_; /*!< 串口到传感器的姿态消息桥接 */
        pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge_; /*!< 串口到传感器的状态消息桥接 */
        
        // 来自 Planner 的命令数据（受 command_mutex_ 保护）
        command_array_t command_array_{};
        RobotCommand command_cache_{};
        Attitude attitude_at_last_frame_{}; // Planner 传来的上一帧姿态
        Attitude attitude_cache_{};
        std::chrono::microseconds plan_period_{0};
        std::chrono::steady_clock::time_point command_start_time_{};
        
        // 来自驱动层的实时数据（受 sensor_mutex_ 保护）
        Attitude latest_attitude_;           /*!< 最新姿态数据 */
        RobotStatus latest_robot_status_;    /*!< 最新机器人状态 */
        
        // 线程同步 - 细粒度锁设计
        std::mutex command_mutex_;  /*!< 保护命令相关数据（来自Planner） */
        std::mutex sensor_mutex_;   /*!< 保护传感器数据（来自驱动层） */
        
        // 性能统计
        fps_counter total_fps{"timedserial_new_fps"};
    };
}

#endif // TIMEDSERIAL_NEW_H
