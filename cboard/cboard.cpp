//
// Created for communication module separation - Cboard_t
// Extracted communication functionality from UartIMU
//

#include "cboard.hpp"

namespace communicationBoard
{
    // Cboard_t 实现
    Cboard_t::Cboard_t(const std::string &device_name) : BasicTask()
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

    void Cboard_t::set_robotcommand(const command_array_t &robotCommands, const Attitude &attitude)
    {
        // 使用互斥锁保护跨线程访问的命令数组和姿态数据
        std::lock_guard<std::mutex> lock(data_mutex);
        command_start_time = std::chrono::steady_clock::now();
        command_array = robotCommands;
        attitude_at_last_frame = attitude;
    }

    // 读取最新命令和姿态数据，基于时间戳进行线性插值
    bool Cboard_t::read_latest_command_and_attitude()
    {
        // 使用互斥锁保护共享数据的并发访问
        std::lock_guard<std::mutex> lock(data_mutex);

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - command_start_time);
        int64_t expectedIndexOne = static_cast<int64_t>(elapsed.count() / send_period.count());

        assert(expectedIndexOne >= 0 && "[cboard] Elapsed time calculation error!");
        if (expectedIndexOne >= CMDARRAYLENGTH - 1)
        {
            // 命令数组已耗尽，保留最后一个元素用于插值计算
            return false;
        }

        std::chrono::microseconds offsetInPeriod = elapsed % send_period;
        command_cache = command_linear_interpolation(command_array[expectedIndexOne], command_array[expectedIndexOne + 1], float(offsetInPeriod.count()) / float(send_period.count()));
        attitude_cache = attitude_at_last_frame;

        return true;
    }

    Cboard_t::~Cboard_t()
    {
        if (imu)
        {
            imu->close();
            delete imu;
        }
        std::cout << "[cboard_submodule] destroyed" << std::endl;
    }

    void Cboard_t::operator()()
    {
        // basictask框架级实现：统一的等待-工作循环
        while (true)
        {
            // basictask框架级实现：等待-启动信号或终止信号
            if (!wait_for_state_change())
            {
                break; // 收到终止信号，退出线程
            }

            // basictask框架级实现：收到启动信号，开始工作循环
            // 本循环内部是具体的任务实现
            unsigned long long frame_index = 0;
            while (isalive())
            {
                auto start_time = std::chrono::high_resolution_clock::now();

                if (imu == nullptr || !imu->is_open())
                {
                    if (_debug)
                    {
                        LOGW_S("[cboard_submodule] Communication not open");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }

                // 发送控制指令
                if (read_latest_command_and_attitude())
                {
                    auto read_time_cost = std::chrono::high_resolution_clock::now() - start_time;
                    if(_debug)
                        if(read_time_cost > std::chrono::microseconds(500))
                            LOGM_S("[cboard_submodule] Cost time: %lld us", (long long)std::chrono::duration_cast<std::chrono::microseconds>(read_time_cost).count());
                    imu->transmit_cmd(
                        attitude_cache.yaw + command_cache.yaw_angle,
                        command_cache.yaw_speed,
                        attitude_cache.pitch + command_cache.pitch_angle,
                        command_cache.pitch_speed,
                        command_cache.distance,
                        static_cast<uint8_t>(command_cache.shoot_mode == ShootMode::COMMON));

                    if (false&&_debug)
                    {
                        LOGM_S("[cboard_submodule][transmit] p-p:%6.2f | p-m:%6.2f | p-s:%6.2f | ps-s:%6.2f | y-p:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f",
                               command_cache.pitch_angle, attitude_cache.pitch,
                               attitude_cache.pitch + command_cache.pitch_angle,
                               command_cache.pitch_speed,
                               command_cache.yaw_angle, attitude_cache.yaw,
                               attitude_cache.yaw + command_cache.yaw_angle,
                               command_cache.yaw_speed);
                    }
                }
                else
                {
                    if (_debug)
                        LOGM_S("[cboard_submodule] No new command to send");
                }

                if (true||_debug)
                {
                    CNT_FPS(total_fps, {});
                    // LOGM_S("[sensor_submodule]Info: Idx = %d, Bytes = %d", data->index, data->frame.size().height * data->frame.size().width);
                }

                auto end_time = std::chrono::high_resolution_clock::now();
                auto sleep_duration = send_period - (end_time - start_time);
                if (sleep_duration > std::chrono::milliseconds(0))
                {
                    std::this_thread::sleep_for(sleep_duration);
                }
                else
                {
                    LOGW_S("[cboard_submodule] sending overrun by %lld ms", (long long)std::chrono::duration_cast<std::chrono::microseconds>(-sleep_duration).count());
                    LOGW_F("[cboard_submodule] sending overrun by %lld us", (long long)std::chrono::duration_cast<std::chrono::microseconds>(-sleep_duration).count());
                }
                
                
                LOGM_F("[cboard]%llu start time: %lld|last time: %lld",frame_index++,
                       static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count()),
                       static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count())
                );


            }
            // basictask框架级实现：工作循环结束（被stop），回到等待状态
        }
    }
}