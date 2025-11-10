//
// Created for communication module separation - Cboard
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

namespace cboard
{
    /**
     * @brief 控制输入的最大范围
     *
     * @param input 输入量
     * @param max 最大值（绝对值）
     * @return float 约化的输出值
     */
    float val_limit(float input, float max);

    /**
     * @brief 用于输出角度滤波的辅助类
     */
    class AngleFilter
    {
    private:
        float angle{0.f};
        bool init{true};

    public:
        /**
         * @brief 重置滤波器
         */
        void reset();

        /**
         * @brief 更新滤波器输入
         *
         * @param input 输入
         * @return float 输出
         */
        float update(float input);

        /**
         * @brief 获取滤波器输出
         *
         * @return float 输出
         */
        float output();
    };

    /**
     * @brief   通讯子模块
     * @details 处理与下位机的串口通讯，包括IMU数据接收和控制指令发送
     */
    class Cboard : public pipeline::BasicTask
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] device_name 串口设备名称
         */
        Cboard(const std::string& device_name);
        virtual ~Cboard();

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        bool operator()() override;

        void get_attitude(Attitude &attitude)
        {
            imu->get_attitude(attitude);
        }

        void get_robotstatus(RobotStatus &robotstatus)
        {
            imu->get_robotstatus(robotstatus);
        }

        bool read_latest_command_and_attitude_optimistic();

        void set_robotcommand(const std::array<RobotCommand, commandArrayLength>& robotCommands, const Attitude& attitude, const std::chrono::milliseconds ctl_period)
        {
            assert(send_period == ctl_period && "Control period mismatch!");
            // 使用互斥锁保护所有共享数据的写入
            std::lock_guard<std::mutex> lock(dataMutex);
            robotCommandArray = robotCommands;
            attitudeAtLastFrame = attitude;
            
            // 关键：收到新数据，重置命令索引，消费者将自动从头开始
            commandIndex = 0; 
        }

    private:
        // 通讯相关成员变量
        UartIMU *imu = nullptr;           /*!< IMU 通讯接口指针 */
        
        // 状态跟踪
        AngleFilter pitch_angle_filter;   /*!< 角度滤波器 */
        static constexpr std::chrono::milliseconds send_period(2);
        size_t commandIndex = 0; // 修复：现在 dataMutex 保护
        static constexpr size_t commandArrayLength = 10;
        
        std::array<RobotCommand, commandArrayLength> robotCommandArray; // 会被跨线程访问，在没有锁保护的情况下，不要读取它，commandCache是安全的本地副本
        RobotCommand commandCache;
        Attitude attitudeAtLastFrame; // 会被跨线程访问，在没有锁保护的情况下，不要读取它，attitudeCache是安全的本地副本
        Attitude attitudeCache;
        
        std::mutex dataMutex;
    };
}

#endif // CBOARD_H