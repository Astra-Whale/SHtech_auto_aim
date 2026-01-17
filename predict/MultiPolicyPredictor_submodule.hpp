/**
 * @file MultiPolicyPredictor.hpp
 * @brief 多策略预测器模块 - 整合跟踪、规划和可视化的主预测系统
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 该模块提供：
 * 1. 集成CoordTransformer、Tracker、Planner的完整预测流程
 * 2. 装甲板筛选和目标管理逻辑
 * 3. 多种可视化显示：实时图像、俯视图仿真
 * 4. 数据输出和调试信息管理
 */

#ifndef PREDICT_LINEAR_PREDICTOR_SUBMODULE_H
#define PREDICT_LINEAR_PREDICTOR_SUBMODULE_H

// modules
#include "common.hpp"
#include "types.hpp"
#include "Tracker.hpp"
#include "Planner.hpp"
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
    /**
     * @brief   多策略预测子模块
     * @details 合并原有 PredictSubModule 和 MultiPolicyPredictor 的功能，
     *          既实现 SubModule 接口用于 pipeline，又包含完整的预测算法实现
     */
    class MultiPolicyPredictorSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief 带参数构造函数
         * @param comm_latency_ 通信延迟时间 (毫秒)
         * @param shoot_latency_ 发射延迟时间 (毫秒)
         * @param debug_ 调试模式标志
         * @param show_ 显示模式标志
         * @param plot_ 绘图模式标志
         * @param adjust_ 参数调整模式标志
         * @details 初始化所有核心组件，设置配置参数
         */
        MultiPolicyPredictorSubModule(int comm_latency_, int shoot_latency_,
                                        double pitch_comp, double yaw_comp, bool disable_vehicle_center_shoot_mode,
                                        bool debug_, bool show_, bool plot_, bool adjust_);
        virtual ~MultiPolicyPredictorSubModule() = default;

        bool should_skip(std::shared_ptr<ThreadDataPack> data) const override;

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    public:
        /// @brief 时间点类型别名
        using TP = std::chrono::high_resolution_clock::time_point;

        /// @brief 相机视野内最大可接受偏航角 (弧度)
        const double max_yaw_accept = 0.85;

    private:
        // === 配置参数 ===
        /// @brief 调试模式标志 - 控制调试信息输出
        bool debug;
        
        /// @brief 显示模式标志 - 控制可视化界面显示
        bool show;
        
        /// @brief 绘图模式标志 - 控制数据绘图输出
        bool plot;
        
        /// @brief 参数调整模式标志 - 控制实时参数调整界面
        bool adjust;

        // === 核心组件 ===
        /// @brief 目标跟踪器 - 执行多模型自适应跟踪
        Tracker tracker;
        
        /// @brief 弹道规划器 - 执行预测和轨迹优化
        Planner planner;

        /// @brief 当前跟踪的观测值 [y, x, z, yaw]
        Eigen::Matrix<double, 4, 1> tracked_measurement;

        /// @brief 当前备选跟踪的观测值 [y, x, z, yaw]
        Eigen::Matrix<double, 4, 1> secondary_tracked_measurement;
        
        /// @brief 当前跟踪的装甲板对象
        bbox_t tracked_armor;

        /// @brief 坐标变换器单例 - 负责坐标系转换
        CoordTransformer& coord_transformer;

    private:
        /**
         * @brief 更新发送给下位机的信息
         * @param plan 预测计划结构体
         * @param send 机器人控制指令结构体
         * @details 将预测结果转换为机器人控制指令，包括云台角度、角速度、射击使能等
         */
        void update_information_to_send(const Plan &plan, RobotCommand &send, 
                                        float attitude_yaw, float attitude_pitch);

        /**
         * @brief 输出数据用于绘图分析
         * @param target 目标跟踪状态
         * @param plan 预测计划
         * @details 输出跟踪和预测的关键数据，用于离线分析和调优
         */
        void output_data_to_plot(const Target &target, const Plan &plan);
        
        /**
         * @brief 显示真实世界视图
         * @param target 目标跟踪状态
         * @param plan 预测计划
         * @param data 线程数据包，包含图像和传感器数据
         * @param show_armor 是否显示装甲板边界框
         * @details 在原始图像上叠加显示：
         *          - 检测到的装甲板边界框
         *          - 估计的车辆中心位置
         *          - 预测的瞄准点
         *          - 跟踪状态信息
         */
        void show_real_world(const Target &target, const Plan &plan, 
                                std::shared_ptr<ThreadDataPack> &data,const Eigen::Matrix3d &R_world2imu, bool show_armor);
        
        /**
         * @brief 显示仿真俯视图
         * @param target 目标跟踪状态
         * @param plan 预测计划
         * @details 显示俯视角度的2D仿真图，包括：
         *          - 车辆中心位置
         *          - 装甲板位置（实测和估计）
         *          - 预测瞄准点
         *          - 装甲板朝向
         */
        void show_sim(const Target &target, const Plan &plan);

        // === 枚举转字符串函数 ===
        /**
         * @brief 跟踪状态转字符串
         * @param x 跟踪状态枚举
         * @return 对应的字符串描述
         */
        std::string TrackingState2String(const TrackingState & x);
        
        /**
         * @brief 瞄准目标类型转字符串
         * @param x 瞄准目标类型枚举
         * @return 对应的字符串描述
         */
        std::string AimedTargetType2String(const AimedTargetType & x);
        
        /**
         * @brief 模型更新类型转字符串
         * @param x 模型更新类型枚举
         * @return 对应的字符串描述
         */
        std::string UpdatingModelType2String(const UpdatingModelType & x);

    public:
        /**
         * @brief 主预测函数 - 执行完整的预测流程
         * @param data 线程数据包，包含：
         *             - 检测到的装甲板列表
         *             - 机器人姿态信息
         *             - 时间戳
         *             - 图像数据
         *             - 机器人状态
         * @details 执行完整的预测流程：
         *          1. 更新坐标变换矩阵
         *          2. 筛选和匹配同ID装甲板
         *          3. 执行目标跟踪更新
         *          4. 生成预测计划
         *          5. 更新控制指令
         *          6. 可视化显示（如果开启）
         */
        void predict(std::shared_ptr<ThreadDataPack> data);
    };
}

#endif // PREDICT_LINEAR_PREDICTOR_SUBMODULE_Htracked_armor