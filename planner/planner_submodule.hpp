//
// EntryStageSubModule
//

#ifndef PLANNER_SUBMODULE_H
#define PLANNER_SUBMODULE_H

// modules
#include "common.hpp"
#include "message_bridge.hpp"

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
        PlannerSubModule(pipeline::bridge::PlannerToSerialBridge &message_bridge);
        virtual ~PlannerSubModule() = default;

        bool should_skip(std::shared_ptr<ThreadDataPack> data) const override;

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    private:
        static constexpr size_t CMDARRAYLENGTH = communicationBoard::Cboard_t::CMDARRAYLENGTH;
        static constexpr std::chrono::microseconds ctl_period = communicationBoard::Cboard_t::send_period;
        using command_array_t = communicationBoard::Cboard_t::command_array_t;
        
        command_array_t generate_command_array(const RobotCommand& command);

        pipeline::bridge::PlannerToSerialBridge &planner_bridge;
        
        static constexpr size_t CMDARRAYLENGTH = 10;
        using command_array_t = std::array<RobotCommand, CMDARRAYLENGTH>;
    };
}

#endif // PLANNER_SUBMODULE_H