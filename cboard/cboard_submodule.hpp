//
// Created for communication module separation - CboardSubModule
// Extracted communication functionality from UartIMU
//

#ifndef CBOARD_CBOARD_SUBMODULE_H
#define CBOARD_CBOARD_SUBMODULE_H

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
    class CboardSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] device_name 串口设备名称
         */
        CboardSubModule(const std::string& device_name);
        virtual ~CboardSubModule();

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        bool process(std::shared_ptr<ThreadDataPack>& data, 
                    pipeline::BasicTask* parent) override;

    private:
        // 通讯相关成员变量
        UartIMU *imu = nullptr;                 /*!< IMU 通讯接口指针 */
        
        // 状态跟踪
        AngleFilter pitch_angle_filter;         /*!< 角度滤波器 */
    };
}

#endif // CBOARD_CBOARD_SUBMODULE_H