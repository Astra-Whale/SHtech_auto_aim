//
// EntryStageSubModule
//

#ifndef ENTRY_STAGE_SUBMODULE_H
#define ENTRY_STAGE_SUBMODULE_H

// modules
#include "common.hpp"

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

namespace entrystage
{
    struct EntryStageConfig : pipeline::ModuleConfig
    {
    };

    /**
     * @brief   入口阶段子模块
     */
    class EntryStageSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         */
        EntryStageSubModule(const EntryStageConfig& config,
                           pipeline::bridge::EntryStageToFoxgloveRobotBridge& robot_bridge,
                           pipeline::bridge::EntryStageToFoxgloveAliveBridge& alive_bridge);
        virtual ~EntryStageSubModule() = default;

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    private:
        EntryStageConfig config_;
        int totalframecounter = 0;              /*!< 总帧数计数器 */
        pipeline::bridge::EntryStageToFoxgloveRobotBridge& robot_bridge_;  /*!< 机器人状态消息桥接 */
        pipeline::bridge::EntryStageToFoxgloveAliveBridge& alive_bridge_;  /*!< 存活信号消息桥接 */
    };
}

#endif // ENTRY_STAGE_SUBMODULE_H