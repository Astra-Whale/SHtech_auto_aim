//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule (Camera only, hardware communication moved to hardware::TimedSerial)
//

#include "sensor_submodule.hpp"

// submodules
#include <chrono>
#include <stdexcept>
#include <thread>
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
    // SensorSubModule 实现
    SensorSubModule::SensorSubModule(const SensorConfig& config, 
                                    const std::string &VideoSource, 
                                    const std::string &flip_image, 
                                    pipeline::bridge::SensorFromSerialAttitudeBridge& attitude_bridge, 
                                    pipeline::bridge::SensorFromSerialRobotStatusBridge& status_bridge) 
        : SubModule(SubModuleName::SENSOR), config_(config), attitude_bridge(attitude_bridge), status_bridge(status_bridge)
    {
        LOGM_S("[sensor_submodule] constructing with video: %s", VideoSource.c_str());
        const bool is_camera_source = (VideoSource == "0");

        // 初始化视频源
        if (is_camera_source)
        {
#ifdef ENABLE_HIKCAM
            video = new HikCamWrapper();
#else
            throw std::runtime_error("camera source requested but ENABLE_HIKCAM is disabled");
#endif
        }
        else
        {
            video = new VideoWrapper(VideoSource);
        }

        if (!video)
        {
            throw std::runtime_error("failed to create video source wrapper");
        }

        // 初始化视频设备
        if (is_camera_source)
        {
            int retry_count = 0;
            while (!video->init())
            {
                ++retry_count;
                if (retry_count == 1 || retry_count % 10 == 0)
                {
                    LOGE_S("[sensor_submodule]Error: Initialize camera stream failed, retrying at 10Hz... (%d)", retry_count);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (retry_count > 0)
            {
                LOGM_S("[sensor_submodule] camera stream initialized after %d retries", retry_count);
            }
        }
        else if (!video->init())
        {
            throw std::runtime_error("failed to initialize video stream: " + VideoSource);
        }
        LOGM_S("[sensor_submodule] video initialized");

        // 设置图像翻转标志
        if (flip_image == "1")
        {
            is_image_input_flipped = true;
            LOGM_S("[sensor_submodule] Input image will be flipped");
        }
        else
        {
            is_image_input_flipped = false;
            // LOGM_S("[sensor_submodule] Input image will not be flipped");
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
    }

    SubModuleResult SensorSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                             const pipeline::BasicTask *parent)
    {
        auto t1 = std::chrono::steady_clock::now();

        // 读取图像
        bool state = video->read(data->frame, config_.debug.log_text);
        data->time = std::chrono::high_resolution_clock::now();

        // 读取imu
        data->attitude = attitude_bridge.get().attitude;
        data->robotstatus = status_bridge.get().robotstatus;

        if (!state)
        {
            LOGE_S("[sensor_submodule]Error: read image fail!");
            if (config_.debug.log_text)
            {
                LOGM_S("[sensor_submodule] Total frames handled: %d", data->index);
                LOGM_S("[sensor_submodule] ReOpen Camera");
            }
            video->close(config_.debug.log_text);
            video->init(config_.debug.log_text);
            // 在失败情况下，返回 false 表示不应该传递到下游
            return SubModuleResult::FAILURE;
        }

        if (data->frame.empty())
        {
            LOGE_S("[sensor_submodule] empty image");
            // 在空图像情况下，也返回 false
            return SubModuleResult::FAILURE;
        }

        if (is_image_input_flipped)
        {
            cv::flip(data->frame, data->frame, -1);
        }

        auto t2 = std::chrono::steady_clock::now();

        // 显示图像（如果需要）
        if (config_.debug.show_image)
        {
            cv::Mat im2show = data->frame.clone();
            cv::imshow("sensor_submodule", im2show);
            cv::waitKey(1);
        }

        // 调试信息
        if (config_.debug.log_text)
        {
            CNT_FPS(total_fps, {});
            // LOGM_S("[sensor_submodule]Info: Idx = %d, Bytes = %d", data->index, data->frame.size().height * data->frame.size().width);
        }

        // 成功处理，返回 true 表示应该传递到下游
        return SubModuleResult::SUCCESS;
    }
}