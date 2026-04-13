//
// EntryStageSubModule
//

#ifndef PLANNER_SUBMODULE_H
#define PLANNER_SUBMODULE_H

// modules
#include "common.hpp"
#include "message_bridge.hpp"
#include "Planner.hpp"

// packages
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace plan
{
    using namespace predict;

    struct PlannerConfig : pipeline::ModuleConfig
    {
        bool plot = false;
    };

    /**
     * @brief   入口阶段子模块
     */
    class PlannerSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] message_bridge 消息桥接对象引用
         */
        PlannerSubModule(const PlannerConfig& config,
                         pipeline::bridge::PlannerToSerialBridge &message_bridge,
                         const std::string planner_param);
        virtual ~PlannerSubModule() = default;

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    private:
        static constexpr size_t CMDARRAYLENGTH = 10;
        static constexpr std::chrono::microseconds plan_period{2000};
        using command_array_t = std::array<RobotCommand, CMDARRAYLENGTH>;
        
        command_array_t generate_command_array(const RobotCommand& command);

        PlannerConfig config_;
        pipeline::bridge::PlannerToSerialBridge &planner_bridge;

        Planner planner;

        /// @brief 坐标变换器单例 - 负责坐标系转换
        CoordTransformer& coord_transformer;

        /**
         * @brief 更新发送给下位机的信息
         * @param plan 预测计划结构体
         * @param send 机器人控制指令结构体
         * @details 将预测结果转换为机器人控制指令，包括云台角度、角速度、射击使能等
         */
        void update_information_to_send(const bool has_fixed_target, const Target &target, const Plan &plan, RobotCommand &send, 
            float attitude_yaw, float attitude_pitch);

        /**
        * @brief 输出数据用于绘图分析
        * @param target 目标跟踪状态
        * @param plan 预测计划
        * @details 输出跟踪和预测的关键数据，用于离线分析和调优
        */
        void output_data_to_plot(const Target &target, const Plan &plan, std::shared_ptr<ThreadDataPack> data);

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
            std::shared_ptr<ThreadDataPack> &data,const Eigen::Matrix3d &R_world2imu);

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
    };
}

#endif // PLANNER_SUBMODULE_H