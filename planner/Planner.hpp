/**
 * @file Planner.hpp
 * @brief 弹道规划器模块 - 实现多策略云台轨迹规划和发射预测
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 该模块提供：
 * 1. 多种策略：无模型跟踪装甲板、装甲板模型跟踪装甲板、整车模型跟踪装甲板、整车模型瞄准车辆中心
 * 2. 基于MPC的云台轨迹优化
 * 3. 弹道补偿和飞行时间计算
 * 4. 智能射击决策和装甲板切换检测
 */

#ifndef _PREDICT_PLANNER_HPP_
#define _PREDICT_PLANNER_HPP_

// modules
#include "common.hpp"
#include "tinympc/tiny_api.hpp"
#include "CoordTransformer.hpp"
#include "math_tools.hpp"

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

using namespace mathutils;

namespace predict
{
    using namespace std::chrono;
    
    /// @brief MPC时间步长 (秒)
    constexpr double DT = 0.01;
    
    /// @brief MPC半视野长度
    constexpr int HALF_HORIZON = 50;
    
    /// @brief MPC完整视野长度
    constexpr int HORIZON = HALF_HORIZON * 2;

    /// @brief 轨迹矩阵类型 [yaw, yaw_vel, pitch, pitch_vel] × HORIZON
    using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

    /**
     * @class Planner
     * @brief 弹道规划器类 - 多策略云台轨迹优化和发射预测
     * @details 实现了四种预测策略：
     *          1. 无模型跟踪装甲板：直接瞄准检测位置
     *          2. 装甲板模型跟踪装甲板：基于装甲板运动轨迹预测
     *          3. 整车模型跟踪装甲板：基于整车旋转模型预测装甲板位置，使用MPC优化云台跟踪轨迹，确保平滑跟踪和准确射击
     *          4. 整车模型瞄准车辆中心：瞄准车辆旋转中心并基于整车模型预测发射窗口
     */
    class Planner
    {
    public:
        /// @brief 时间点类型别名
        using TP = std::chrono::high_resolution_clock::time_point;

        // === 云台物理限制参数 ===
        /// @brief 最大偏航角加速度
        const double max_yaw_acc = 50;
        
        /// @brief 最大俯仰角加速度
        const double max_pitch_acc = 100;

        // === 装甲板切换检测参数 ===
        /// @brief 装甲板切换时间间隔 (秒)
        const int armor_jump_interval = 0.1f;

        // === 旋转速度分类阈值 ===
        /// @brief 慢速旋转上限 (弧度/秒)
        const double slow_rotation_upper_bound = 2.1;
        
        /// @brief 中速旋转下限 (弧度/秒)
        const double medium_rotation_lower_bound = 1.57;
        
        /// @brief 中速旋转上限 (弧度/秒)
        const double medium_rotation_upper_bound = 7.33;
        
        /// @brief 高速旋转下限 (弧度/秒)
        const double fast_rotation_lower_bound = 6.28;

        // === 射击决策参数 ===        
        /// @brief 射击精度阈值 (弧度) - 约等于atan(0.05/0.4)
        const double fire_threshold = 0.05; // 0.125;

        /// @brief 位置变化阈值 (米)，用于检测装甲板切换
        const double same_position_threshold = 0.2;

        const double shoot_interval = 0.2;

    private:
        // === 配置参数 ===
        /// @brief 通信延迟 (秒)
        double comm_latency;

        /// @brief 单次发射延迟 (秒)
        double single_shoot_latency;

        /// @brief 连续射击延迟 (秒)
        double continue_shoot_latency;

        /// @brief 射击时机偏移（MPC步数）(与shoot_latency配合使用)
        int shoot_offset = 2;

        double pitch_comp = 0;

        double yaw_comp = 0;

        /// @brief 轨迹跟踪一致性阈值 (弧度)
        double same_trace_threshold = 0.003;

        bool disable_vehicle_center_shoot_mode = true;

        bool disable_armor_with_vehicle_shoot_mode = false;

        bool consider_air_resistence = false;

        /// @brief 调试模式标志
        bool debug;

        // === 状态变量 ===
        /// @brief 当前预测计划
        Plan plan;

        /// @brief 偏航轴MPC求解器指针
        TinySolver * yaw_solver_;
        
        /// @brief 俯仰轴MPC求解器指针
        TinySolver * pitch_solver_;

        /// @brief 上次瞄准的装甲板位置，用于检测装甲板切换
        Eigen::Matrix<double, 3, 1> last_shooted_armor_pos;

        /// @brief 装甲板切换标志
        bool armor_jump;
        
        /// @brief 装甲板切换时间戳
        std::chrono::_V2::system_clock::time_point armor_jump_tp;

        /// @brief 原始目标偏航角（未经MPC优化）
        double target_yaw_raw;
        
        /// @brief 原始目标俯仰角（未经MPC优化）
        double target_pitch_raw;

        /// @brief 坐标变换器单例 - 负责坐标系转换
        CoordTransformer& coord_transformer;

        /// @brief 允许发弹时间戳
        std::chrono::_V2::system_clock::time_point fire_enable_tp;

    private:
        /**
         * @brief 设置偏航轴MPC求解器
         * @details 配置状态空间模型、代价函数和约束条件
         */
        void setup_yaw_solver();
        
        /**
         * @brief 设置俯仰轴MPC求解器
         * @details 配置状态空间模型、代价函数和约束条件
         */
        void setup_pitch_solver();

        /**
         * @brief 预测最近的装甲板位置
         * @param target 目标跟踪状态
         * @param delay 预测时间延迟 (秒)
         * @return 预测的装甲板位置 [x, y, z]
         * @details 基于整车模型预测四个装甲板位置，返回距离最近的一个
         */
        Eigen::Matrix<double, 3, 1> predict_closest_armor(const Target &target, const double delay, int &armor_index);

        /**
         * @brief 计算云台目标角度
         * @param aimed_armor_pos 瞄准的装甲板位置
         * @param bullet_speed 弹丸速度
         * @param attitude_yaw 机器人当前偏航角
         * @param attitude_pitch 机器人当前俯仰角
         * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
         * @return [target_yaw, target_pitch] 云台目标角度
         * @details 考虑弹道下降，计算云台应达到的偏航角和俯仰角
         */
        Eigen::Matrix<double, 2, 1> cal_gimbal_target(Eigen::Matrix<double, 3, 1> aimed_armor_pos, 
                                                        const float bullet_speed, 
                                                        const double attitude_yaw, const double attitude_pitch,
                                                        const Eigen::Matrix3d &R_world2imu);

        /**
         * @brief 生成MPC参考轨迹
         * @param target 目标跟踪状态
         * @param total_delay 总延迟时间
         * @param yaw0 初始偏航角偏移
         * @param bullet_speed 弹丸速度
         * @param attitude_yaw 机器人偏航角
         * @param attitude_pitch 机器人俯仰角
         * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
         * @return 完整的参考轨迹矩阵
         * @details 生成HORIZON长度的参考轨迹，包含偏航角、偏航角速度、俯仰角、俯仰角速度
         */
        Trajectory get_trajectory(const Target &target, const double total_delay, const double yaw0, const double bullet_speed, 
                                    const double attitude_yaw, const double attitude_pitch,
                                    const Eigen::Matrix3d &R_world2imu);

        /**
         * @brief 更新瞄准目标类型
         * @param target 目标跟踪状态
         * @param rotation_speed 目标旋转速度
         * @details 根据跟踪状态和旋转速度自动选择最适合的云台策略
         */
        void update_aimed_target_type(const Target &target, double rotation_speed);

        /**
         * @brief 计算目标角速度
         * @param x_state X坐标状态 [x, vx]
         * @param y_state Y坐标状态 [y, vy]
         * @param z_state Z坐标状态 [z, vz]
         * @return [yaw_speed, pitch_speed] 目标角速度
         * @details 从目标直角坐标速度计算对应的云台角速度
         */
        Eigen::Matrix<double, 2, 1> cal_target_speed(const Eigen::Matrix<double, 2, 1> &x_state, 
                                                        const Eigen::Matrix<double, 2, 1> &y_state, 
                                                        const Eigen::Matrix<double, 2, 1> &z_state);

    public:
        /**
         * @brief 默认构造函数
         */
        Planner() = default;
        
        /**
         * @brief 带参数构造函数
         * @param comm_latency_ 通信延迟时间 (s)
         * @param shoot_latency_ 发射延迟时间 (s)
         * @param debug_ 调试模式标志
         */
        explicit Planner(const std::string planner_param, bool debug_);

        /**
         * @brief 重置规划器状态
         * @details 清空瞄准位置和装甲板切换状态
         */
        void planner_reset();
        
        /**
         * @brief 初始化瞄准目标
         * @details 设置初始瞄准模式为无模型预测
         */
        void aim_target_init();

        /**
         * @brief 获取当前预测计划
         * @return 计划结构体的常引用
         */
        const Plan& get_plan();
        
        /**
         * @brief 制定预测计划
         * @param target 目标跟踪状态
         * @param bullet_speed 弹丸速度
         * @param attitude_yaw 机器人偏航角
         * @param attitude_pitch 机器人俯仰角
         * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
         * @param tp 当前时间戳
         * @return 生成的预测计划
         * @details 根据目标状态选择预测策略，生成包含云台角度、角速度和射击决策的完整计划
         */
        const Plan& make_plan(const Target &target, const float bullet_speed,
                                const double attitude_yaw, const double attitude_pitch, 
                                const Eigen::Matrix3d &R_world2imu, const TP &tp);

        /**
         * @brief 整车状态转换为观测值
         * @param state 整车状态向量 [y, vy, x, vx, z, vz, yaw, vyaw, r]
         * @return 观测向量 [ya, xa, z, yaw] - 装甲板位置和偏航角
         * @details 从整车中心状态计算当前装甲板的观测值
         */
        Eigen::Matrix<double, 4, 1> whole_state_2_measurement(const Eigen::Matrix<double, 11, 1> &state);

    }; // class Planner

} // namespace predict

#endif // _PREDICT_PLANNER_HPP_
