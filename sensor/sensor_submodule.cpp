//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule
//

#include "sensor_submodule.hpp"

//submodules
#include <video/video_wrapper.hpp>
#ifdef ENABLE_HIKCAM
#warning ENABLE_HIKCAM 
#include <hikcam/hikcam_wrapper.hpp>
#else
#warning 
#warning DISABLE_HIKCAM 
#endif

namespace sensor
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

    // SensorSubModule 实现
    SensorSubModule::SensorSubModule(const std::string& VideoSource, const std::string& ImuSource, 
                                     const std::string& port, const std::string& flip_image) : SubModule()
    {
        LOGM_S("[sensor_submodule] constructing with video: %s, IMU: %s, port: %s", 
               VideoSource.c_str(), ImuSource.c_str(), port.c_str());
        
        // 初始化视频源
        if (VideoSource == "0")
        {
            #ifdef ENABLE_HIKCAM
                video = new HikCamWrapper();
            #else
                LOGE_S("[sensor_submodule] hikcam not enabled!");
            #endif
        }
        else
        {
            video = new VideoWrapper(VideoSource);
        }
        
        // 初始化视频设备
        while (!video->init())
        {
            LOGE_S("[sensor_submodule]Error: Initialize video stream failed");
        }
        LOGM_S("[sensor_submodule] video initialized");
        
        // 初始化IMU
        LOGM_S("[sensor_submodule] IMU input from %s", ImuSource.c_str());
        imu = nullptr;
        if (ImuSource == "UART")
        {
            imu = new UartIMU(port);
        }
        if (imu == nullptr || !imu->init())
        {
            LOGE_S("[sensor_submodule]Error: IMU init failed");
        }
        else
        {
            LOGM_S("[sensor_submodule] IMU initialized");
        }
        
        // 设置图像翻转标志
        if (flip_image == "1")
        {
            is_image_input_flipped = true;
            LOGW_S("[sensor_submodule] Input image will be flipped");
        }
        else 
        {
            is_image_input_flipped = false;
            LOGW_S("[sensor_submodule] Input image will not be flipped");
        }
        
        LOGM_S("[sensor_submodule] construction completed");
    }

    SensorSubModule::~SensorSubModule()
    {
        if (video)
        {
            video->close();
            delete video;
        }
        if (imu)
        {
            imu->close();
            delete imu;
        }
    }

    void SensorSubModule::init()
    {
        LOGM_S("[sensor_submodule] init - starting devices");
        
        // 启动 IMU
        if (imu != nullptr)
        {
            imu->start();
            LOGM_S("[sensor_submodule] IMU started");
        }
        
        SubModule::init();
        LOGM_S("[sensor_submodule] ready");
    }

    bool SensorSubModule::process(std::shared_ptr<ThreadDataPack>& data, 
                                  pipeline::BasicTask* parent)
    {
        if (!_init)
        {
            LOGE_S("[sensor_submodule]Error: process before init.");
            return false;
        }

        auto t1 = std::chrono::steady_clock::now();
        
        // 读取图像
        bool state = video->read(data->frame, _debug);
        data->index = totalFrameCounter++;
        data->time = std::chrono::high_resolution_clock::now();

        if (!state)
        {
            if (_debug)
            {
                LOGE_S("[sensor_submodule]Error: read image fail!");
                LOGM_S("[sensor_submodule] Total frames handled: %d", totalFrameCounter);
                LOGM_S("[sensor_submodule] ReOpen Camera");
            }
            video->close(_debug);
            video->init(_debug);
            // 在失败情况下，返回 false 表示不应该传递到下游
            return false;
        }

        if (data->frame.empty())
        {
            LOGW_S("[sensor_submodule] empty image");
            // 在空图像情况下，也返回 false
            return false;
        }

        if (is_image_input_flipped)
        {
            cv::flip(data->frame, data->frame, -1);
        }
        
        auto t2 = std::chrono::steady_clock::now();

        // 处理 IMU 和发送控制指令
        if (imu != nullptr && imu->is_open())
        {
            auto &send = data->robotcommand;
            auto &_attitude = data->attitude;
            imu->transmit_cmd(
                _attitude.yaw + val_limit(send.yaw_angle, 10),
                pitch_angle_filter.update(_attitude.pitch + val_limit(send.pitch_angle, 10)),
                send.yaw_speed, 
                send.pitch_speed, 
                send.distance,
                static_cast<uint8_t>(send.shoot_mode == ShootMode::COMMON)
            );
        }
        else
        {
            pitch_angle_filter.reset();
        }
            
        if (_debug)
        {
            auto &send = data->robotcommand;
            auto &_attitude = data->attitude;
            LOGM_S("[sensor_submodule][transmit] p-p:%6.2f | p-m:%6.2f | p-s:%6.2f | y-p:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f",
                    send.pitch_angle, _attitude.pitch,
                    pitch_angle_filter.output(),
                    send.yaw_angle, _attitude.yaw,
                    _attitude.yaw + val_limit(send.yaw_angle, 10),
                    send.yaw_speed);
        }

        // 读取 IMU 数据
        if (imu != nullptr)
        {
            imu->get_attitude(data->attitude);
            imu->get_robotstatus(data->robotstatus);
        }
        
        auto t3 = std::chrono::steady_clock::now();

        // 显示图像（如果需要）
        if (_show)
        {
            cv::Mat im2show = data->frame.clone();
            cv::imshow("sensor_submodule", im2show);
            cv::waitKey(1);
        }

        // 调试信息
        if (_debug)
        {
            CNT_FPS(total_fps, {});
            // LOGM_S("[sensor_submodule]Info: Idx = %d, Bytes = %d", data->index, data->frame.size().height * data->frame.size().width);
        }
        
        auto t4 = std::chrono::steady_clock::now();
        // LOGM_S(
        //     "SensorSubModule Read %.2lfms Process %.2lfms Show %.2lfms", 
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000,
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()*1000
        // );
        
        // 成功处理，返回 true 表示应该传递到下游
        return true;
    }
}