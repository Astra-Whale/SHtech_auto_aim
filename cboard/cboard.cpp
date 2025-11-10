//
// Created for communication module separation - Cboard
// Extracted communication functionality from UartIMU
//

#include "cboard.hpp"
#include <functional>
#include <chrono>

namespace cboard
{
    /**
     * @brief 控制输入的最大范围
     *
     * @param input 输入量
     * @param max 最大值（绝对值）
     * @return float 约化的输出值
     */
    float val_limit(float input, float max)
    {
        return input < max ? input > -max ? input : -max : max;
    }

    // AngleFilter 实现
    void AngleFilter::reset()
    {
        angle = 0.f;
        init = true;
    }

    float AngleFilter::update(float input)
    {
        if (init)
        {
            angle = input;
            init = false;
        }
        else
        {
            angle = angle * 0.9f + input * 0.1f;
        }
        return angle;
    }

    float AngleFilter::output()
    {
        return angle;
    }

    // Cboard 实现
    Cboard::Cboard(const std::string& device_name) : BasicTask()
    {
        LOGM_S("[cboard_submodule] constructing with device: %s", device_name.c_str());
        
        // 初始化IMU通讯
        imu = new UartIMU(device_name);
        if (imu == nullptr || !imu->init())
        {
            LOGE_S("[cboard_submodule] Failed to initialize IMU communication");
        }
        else
        {
            LOGM_S("[cboard_submodule] IMU communication initialized successfully");
            // 立即启动通讯
            imu->start();
        }
    }

    bool Cboard::read_latest_command_and_attitude_optimistic()
    {
        // 修复：使用互斥锁保护所有共享数据的读取和 commandIndex 的修改
        std::lock_guard<std::mutex> lock(dataMutex);

        // 如果上次已经读完所有命令，返回false，将索引重置到开头等待
        if(commandIndex >= commandArrayLength)
        {
            return false; // 本周期没有新命令要发送
        }
        
        // 修复：在锁内安全地读取数据
        commandCache = robotCommandArray[commandIndex];
        attitudeCache = attitudeAtLastFrame;

        // 修复：在锁内安全地推进索引
        commandIndex++;
        
        return true;
    }

    Cboard::~Cboard()
    {
        if (imu)
        {
            imu->close();
            delete imu;
        }
    }

    void Cboard::operator()()
    {
        // 统一的等待-工作循环
        while (true)
        {
            // 等待启动信号或终止信号
            if (!wait_for_state_change())
            {
                break;  // 收到终止信号，退出线程
            }
            
            // 收到启动信号，开始工作循环
            while (isalive())
            {
                auto start_time = std::chrono::high_resolution_clock::now();

                if (!is_open())
                {
                    pitch_angle_filter.reset();
                    if (_debug)
                    {
                        LOGW_S("[cboard_submodule] Communication not open");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }

                // 发送控制指令
                if(read_latest_command_and_attitude_optimistic())
                {
                    imu->transmit_cmd(
                        attitudeCache.yaw + val_limit(commandCache.yaw_angle, 10),
                        commandCache.yaw_speed,
                        pitch_angle_filter.update(attitudeCache.pitch + val_limit(commandCache.pitch_angle, 10)),
                        commandCache.pitch_speed, 
                        commandCache.distance,
                        static_cast<uint8_t>(commandCache.shoot_mode == ShootMode::COMMON)
                    );

                    if (_debug)
                    {
                        LOGM_S("[cboard_submodule][transmit] p-p:%6.2f | p-m:%6.2f | p-s:%6.2f | y-p:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f",
                                commandCache.pitch_angle, attitudeCache.pitch,
                                pitch_angle_filter.output(),
                                commandCache.yaw_angle, attitudeCache.yaw,
                                attitudeCache.yaw + val_limit(commandCache.yaw_angle, 10),
                                commandCache.yaw_speed);
                    }
                }
                else
                {
                    LOGM_S("[cboard_submodule] No new command to send");
                }
                auto end_time = std::chrono::high_resolution_clock::now();
                auto sleep_duration = send_period - (end_time - start_time);
                if(sleep_duration > std::chrono::milliseconds(0))
                {
                    std::this_thread::sleep_for(sleep_duration);
                }
                else
                {
                    LOGW_S("[cboard_submodule] sending overrun by %lld ms", std::chrono::duration_cast<std::chrono::milliseconds>(-sleep_duration).count());
                }

            }
            
            // 工作循环结束（被stop），回到等待状态
        }        
        
    }
}