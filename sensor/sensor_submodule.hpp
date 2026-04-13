//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule (Camera only, hardware communication moved to hardware::TimedSerial)
//

#ifndef SENSOR_SENSOR_SUBMODULE_H
#define SENSOR_SENSOR_SUBMODULE_H

// submodules
#include "cam_wrapper.hpp"

// modules
#include "common.hpp"

// packages
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

namespace sensor
{
    struct SensorConfig : pipeline::ModuleConfig
    {
    };

    /**
     * @brief   传感器子模块
     * @details 包装原有 Sensor 逻辑为 SubModule，专注于相机数据处理
     *          通讯功能已移至 hardware::TimedSerial 模块
     */
    class SensorSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] VideoSource 视频源路径
         * @param[in] flip_image 是否翻转图像
         */
        SensorSubModule(const SensorConfig& config, const std::string& VideoSource, const std::string& flip_image, pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge, pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge); 
        virtual ~SensorSubModule();

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    private:
        SensorConfig config_;
        // 传感器相关成员变量
        WrapperHead *video = nullptr;           /*!< 视频输入接口指针 */
        bool is_image_input_flipped = false;    /*!< 标记输入图像是否需要翻转 */
        
        // 状态跟踪
        fps_counter total_fps{"sensor_fps"};    /*!< FPS计数器 */

        pipeline::bridge::SensorFromSerialAttitudeBridge &attitude_bridge; /*!< 串口到传感器的姿态消息桥接 */
        pipeline::bridge::SensorFromSerialRobotStatusBridge &status_bridge; /*!< 串口到传感器的状态消息桥接 */
    };
}

#endif // SENSOR_SENSOR_SUBMODULE_H