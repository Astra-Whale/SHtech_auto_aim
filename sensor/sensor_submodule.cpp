//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule (Camera only, hardware communication moved to hardware::TimedSerial)
//

#include "sensor_submodule.hpp"

// submodules
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
    SensorSubModule::SensorSubModule(const std::string &VideoSource, 
                                    const std::string &flip_image, 
                                    pipeline::bridge::SensorFromSerialAttitudeBridge& attitude_bridge, 
                                    pipeline::bridge::SensorFromSerialRobotStatusBridge& status_bridge) 
        : SubModule(SubModuleName::SENSOR), attitude_bridge(attitude_bridge), status_bridge(status_bridge)
    {
        LOGM_S("[sensor_submodule] constructing with video: %s", VideoSource.c_str());
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

    bool SensorSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const
    {
        return false;
    }

    SubModuleResult SensorSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                             const pipeline::BasicTask *parent)
    {
        auto t1 = std::chrono::steady_clock::now();

        // 读取图像
        bool state = video->read(data->frame, _debugprint);
        data->time = std::chrono::high_resolution_clock::now();

        // 读取imu
        data->attitude = attitude_bridge.get().attitude;
        data->robotstatus = status_bridge.get().robotstatus;

        if (!state)
        {
            LOGE_S("[sensor_submodule]Error: read image fail!");
            if (_debugprint)
            {
                LOGM_S("[sensor_submodule] Total frames handled: %d", data->index);
                LOGM_S("[sensor_submodule] ReOpen Camera");
            }
            video->close(_debugprint);
            video->init(_debugprint);
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
        if (_imgshow)
        {
            cv::Mat im2show = data->frame.clone();
            cv::imshow("sensor_submodule", im2show);
            cv::waitKey(1);
        }

        // 调试信息
        if (_debugprint)
        {
            CNT_FPS(total_fps, {});
            // LOGM_S("[sensor_submodule]Info: Idx = %d, Bytes = %d", data->index, data->frame.size().height * data->frame.size().width);
        }

        // 成功处理，返回 true 表示应该传递到下游
        return SubModuleResult::SUCCESS;
    }
}