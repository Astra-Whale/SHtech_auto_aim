//
// Created for pipeline refactor - SensorSubModule
// Wraps original Sensor logic as SubModule
//

#ifndef SENSOR_SENSOR_SUBMODULE_H
#define SENSOR_SENSOR_SUBMODULE_H

// submodules
#include "cam_wrapper.hpp"
#include "UartIMU/uartimu.hpp"

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
     * @brief   传感器子模块
     * @details 包装原有 Sensor 逻辑为 SubModule，用于 CompositeTask
     */
    class SensorSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] VideoSource 视频源路径
         * @param[in] ImuSource IMU源类型
         * @param[in] port 通信端口
         * @param[in] flip_image 是否翻转图像
         */
        SensorSubModule(const std::string& VideoSource, const std::string& ImuSource, 
                        const std::string& port, const std::string& flip_image);
        virtual ~SensorSubModule();

        /**
         * @brief   子模块初始化
         * @details 只负责启动设备和设置初始化标志，所有实际初始化工作在构造函数中完成
         */
        void init();

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        bool process(std::shared_ptr<ThreadDataPack>& data, 
                    pipeline::BasicTask* parent) override;

    private:
        // 传感器相关成员变量
        WrapperHead *video = nullptr;          /*!< 视频输入接口指针 */
        UartIMU *imu = nullptr;                /*!< IMU 输入接口指针 */
        bool is_image_input_flipped = false;    /*!< 标记输入图像是否需要翻转 */
        
        // 状态跟踪
        int totalFrameCounter = 0;              /*!< 总帧数计数器 */
        AngleFilter pitch_angle_filter;         /*!< 角度滤波器 */
        fps_counter total_fps{"sensor_fps"};    /*!< FPS计数器 */
    };
}

#endif // SENSOR_SENSOR_SUBMODULE_H