//
// Created for hardware communication module - TimedSerial
// Handles timed serial communication with lower machines
//

#ifndef CBOARD_H
#define CBOARD_H

// submodules
#include "UartIMU/uartimu.hpp"

// modules
#include "common.hpp"
#include "message_bridge.hpp"

// packages
#include <stdint.h>
#include <string>
#include <functional>
#include <chrono>

namespace hardware
{
    /**
     * @brief   串口定时通讯子模块
     * @details 处理与下位机的串口通讯，包括IMU数据接收和控制指令发送
     */
    class TimedSerial : public pipeline::BasicTask
    {
    public:

        /**
         * @brief   构造函数
         * @param[in] device_name 串口设备名称
         * @param[in] message_bridge 消息桥接对象引用
         */
        TimedSerial(const std::string &device_name, pipeline::bridge::PlannerToSerialBridge &message_bridge);
        virtual ~TimedSerial();

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        void operator()() override;

        void get_attitude(Attitude &attitude)
        {
            imu->get_attitude(attitude);
        }

        void get_robotstatus(RobotStatus &robotstatus)
        {
            imu->get_robotstatus(robotstatus);
        }

    private:
        bool read_latest_command_and_attitude();
        
        /**
         * @brief   处理来自 Planner 的命令消息（回调函数）
         * @param[in] msg 包含命令数组和姿态的消息
         */
        void handle_planner_message(const pipeline::bridge::PlannerToSerialMessage &msg);

        static constexpr size_t CMDARRAYLENGTH = 10;
        static constexpr std::chrono::microseconds send_period{2000};
        using command_array_t = std::array<RobotCommand, CMDARRAYLENGTH>;
        
        // 通讯相关成员变量
        UartIMU *imu = nullptr; /*!< IMU 通讯接口指针 */
        pipeline::bridge::PlannerToSerialBridge &planner_bridge; /*!< 消息桥接引用 */
        
        command_array_t command_array; // 会被跨线程访问，在没有锁保护的情况下，不要读取它，commandCache是安全的本地副本
        RobotCommand command_cache;
        Attitude attitude_at_last_frame; // 会被跨线程访问，在没有锁保护的情况下，不要读取它，attitudeCache是安全的本地副本
        Attitude attitude_cache;
        std::chrono::microseconds plan_period; // planner模块的控制周期，原理上会被跨线程访问，但目前只在初始化时写入一次
        std::chrono::steady_clock::time_point command_start_time;
        std::mutex data_mutex;
        fps_counter total_fps{"cboard_fps"};
    };
}

#endif // CBOARD_H