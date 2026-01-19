/**
 * @file types.hpp
 * @brief 预测系统类型定义模块 - 定义预测和跟踪相关的数据结构和枚举
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 该模块定义：
 * 1. 目标瞄准类型枚举
 * 2. 跟踪状态枚举
 * 3. 模型更新类型枚举
 * 4. 预测计划结构体
 * 5. 目标跟踪结构体
 */

#ifndef _PREDICT_TYPES_HPP_
#define _PREDICT_TYPES_HPP_

// packages
#include <iostream>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <cfloat>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    /**
     * @enum AimedTargetType
     * @brief 瞄准目标类型枚举 - 定义不同的预测策略模式
     * @details 根据目标运动状态选择不同的瞄准策略
     */
    enum class AimedTargetType : uint8_t
    {
        /// @brief 无目标
        NONE,
        
        /// @brief 无模型跟踪装甲板
        ARMOR_WITH_NO_MODEL,
        
        /// @brief 装甲板模型跟踪装甲板
        ARMOR_WITH_ARMOR_MODEL,
        
        /// @brief 整车模型跟踪装甲板
        ARMOR_WITH_VEHICLE_MODEL,
        
        /// @brief 整车模型瞄准车辆中心
        VEHICLE_CENTER_WITH_VEHICLE_MODEL,
    };

    /**
     * @struct Plan
     * @brief 预测计划结构体 - 包含云台控制和射击决策的完整信息
     * @details 由Planner模块生成，包含目标预测位置、云台角度控制参数和射击使能
     */
    struct Plan
    {
        /// @brief 当前瞄准目标类型
        AimedTargetType aimed_target_type;

        /// @brief 预测的装甲板瞄准位置 [x, y, z] (米)，世界坐标系
        Eigen::Matrix<double, 3, 1> aimed_armor_pos;

        /// @brief 目标偏航角 (弧度)，云台应达到的偏航角度
        double target_yaw;
        
        /// @brief 目标偏航角速度 (弧度/秒)，云台偏航轴应达到的角速度
        double target_yaw_speed;

        /// @brief 目标偏航角速度 (弧度/秒2)，云台偏航轴应达到的角加速度
        double target_yaw_acc;

        /// @brief 目标俯仰角 (弧度)，云台应达到的俯仰角度
        double target_pitch;
        
        /// @brief 目标俯仰角速度 (弧度/秒)，云台俯仰轴应达到的角速度
        double target_pitch_speed;

        /// @brief 目标俯仰角速度 (弧度/秒2)，云台俯仰轴应达到的角加速度
        double target_pitch_acc;

        /// @brief 射击使能标志 (0=禁止, 1=允许, 2=下位机决策)
        int fire_enable;

        /// @brief 目标距离 (米)，目标的直线距离
        double target_distance;
    };

    /**
     * @enum TrackingState
     * @brief 跟踪状态枚举 - 描述目标跟踪器的当前工作状态
     * @details 跟踪器状态机的四个状态，控制不同阶段的跟踪行为
     */
    enum class TrackingState : uint8_t
    {
        /// @brief 空闲状态 - 未发现目标，等待检测
        IDLE,
        
        /// @brief 检测状态 - 发现目标但尚未稳定跟踪
        DETECTING,
        
        /// @brief 跟踪状态 - 正在稳定跟踪目标
        TRACKING,
        
        /// @brief 暂时丢失状态 - 目标暂时消失，保持预测
        TEMP_LOST,
    };

    /**
     * @enum UpdatingModelType
     * @brief 模型更新类型枚举 - 指定当前使用的滤波模型类型
     * @details 根据目标运动速度动态选择使用的滤波器类型：
     *          - 低速时使用装甲板模型
     *          - 高速时使用整车模型
     *          - 中速时两个模型同时使用
     */
    enum class UpdatingModelType : uint8_t
    {
        /// @brief 仅更新装甲板运动模型（适用于低速运动）
        ARMOR_MODEL,
        
        /// @brief 仅更新整车运动模型（适用于高速旋转）
        VEHICLE_MODEL,
        
        /// @brief 同时更新两种模型（适用于中速过渡阶段）
        BOTH,
    };

    /**
     * @struct Target
     * @brief 目标跟踪结构体 - 包含目标的完整状态信息和滤波器状态
     * @details 存储跟踪器的所有状态变量，包括EKF和KF的状态估计
     */
    struct Target
    {
        /// @brief 当前跟踪器状态
        TrackingState predictor_state;
        
        /// @brief 当前模型更新类型
        UpdatingModelType updating_model_type;

        /// @brief 装甲板切换计数器（0或1，标识当前跟踪的装甲板）
        int ab_counter;

        /// @brief 整车模型可信标志
        bool vehicle_model_trust;
        
        /// @brief 整车状态向量 [y, vy, x, vx, z, vz, yaw, vyaw, r] (9x1)
        /// @details y,x,z: 车辆中心位置；vy,vx,vz: 速度；yaw,vyaw: 装甲板偏航角和角速度；r: 旋转半径
        /// 坐标使用世界坐标系，装甲板偏航角坐标系为：正对世界坐标系y轴为0度，以世界坐标系z轴负半轴为正方向旋转
        Eigen::Matrix<double, 11, 1> tracked_state;
        
        /// @brief 整车模型观测向量 [y, x, z, yaw] (4x1)
        Eigen::Matrix<double, 4, 1> tracked_measurement;
        
        /// @brief 另一对装甲板的旋转半径 (米)
        double another_r;
        
        /// @brief 装甲板高度差 (米)，用于处理上下装甲板
        double dz;

        // === 装甲板模型的卡尔曼滤波器状态 ===
        /// @brief 偏航角KF状态 [yaw, yaw_velocity] (2x1)
        Eigen::Matrix<double, 2, 1> yaw_state;
        
        /// @brief 偏航角KF观测 [yaw] (1x1)
        Eigen::Matrix<double, 1, 1> yaw_measurement;

        /// @brief X坐标KF状态 [x, vx] (2x1)
        Eigen::Matrix<double, 2, 1> armor_x_state;
        
        /// @brief X坐标KF观测 [x] (1x1)
        Eigen::Matrix<double, 1, 1> armor_x_measurement;

        /// @brief Y坐标KF状态 [y, vy] (2x1)
        Eigen::Matrix<double, 2, 1> armor_y_state;
        
        /// @brief Y坐标KF观测 [y] (1x1)
        Eigen::Matrix<double, 1, 1> armor_y_measurement;

        /// @brief Z坐标KF状态 [z, vz] (2x1)
        Eigen::Matrix<double, 2, 1> armor_z_state;
        
        /// @brief Z坐标KF观测 [z] (1x1)
        Eigen::Matrix<double, 1, 1> armor_z_measurement;
    };
} // namespace predict

#endif // _PREDICT_TYPES_HPP_
