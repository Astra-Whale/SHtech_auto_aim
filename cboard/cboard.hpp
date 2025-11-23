//
// Created for communication module separation - Cboard_t
// Extracted communication functionality from UartIMU
//

#ifndef CBOARD_H
#define CBOARD_H

// submodules
#include "UartIMU/uartimu.hpp"

// modules
#include "common.hpp"

// packages
#include <stdint.h>
#include <string>
#include <functional>
#include <chrono>

namespace communicationBoard
{
    /**
     * @brief   通讯子模块
     * @details 处理与下位机的串口通讯，包括IMU数据接收和控制指令发送
     */
    class Cboard_t : public pipeline::BasicTask
    {
    public:
        static constexpr size_t CMDARRAYLENGTH = 10;
        static constexpr std::chrono::microseconds send_period{2000};
        using command_array_t = std::array<RobotCommand, CMDARRAYLENGTH>;

        /**
         * @brief   构造函数
         * @param[in] device_name 串口设备名称
         */
        Cboard_t(const std::string &device_name);
        virtual ~Cboard_t();

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


        void set_robotcommand(const command_array_t &robotCommands, const Attitude &attitude);

    private:
        bool read_latest_command_and_attitude();
        
        // 通讯相关成员变量
        UartIMU *imu = nullptr; /*!< IMU 通讯接口指针 */
        
        command_array_t command_array; // 会被跨线程访问，在没有锁保护的情况下，不要读取它，commandCache是安全的本地副本
        RobotCommand command_cache;
        Attitude attitude_at_last_frame; // 会被跨线程访问，在没有锁保护的情况下，不要读取它，attitudeCache是安全的本地副本
        Attitude attitude_cache;
        std::chrono::steady_clock::time_point command_start_time;
        std::mutex data_mutex;
    };
}

#endif // CBOARD_H