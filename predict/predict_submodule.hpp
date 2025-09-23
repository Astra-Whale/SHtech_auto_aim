//
// Created for pipeline refactor - PredictSubModule
// Wraps original Predict logic as SubModule
//

#ifndef PREDICT_PREDICT_SUBMODULE_H
#define PREDICT_PREDICT_SUBMODULE_H

// submodules
#include "tools.hpp"
#include "StaticPredictor.hpp"
#include "LinearPredictor.hpp"

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

namespace predict
{
    /**
     * @brief   预测子模块
     * @details 包装原有 Predict 逻辑为 SubModule，用于 CompositeTask
     */
    class PredictSubModule : public pipeline::SubModule
    {
    public:
        PredictSubModule() : SubModule() {}
        virtual ~PredictSubModule() = default;

        /**
         * @brief   子模块初始化
         * @param[in] camera_param 相机参数文件路径
         * @param[in] latency 通信延迟（毫秒）
         */
        void init(const std::string camera_param, int latency = 20);

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        bool process(std::shared_ptr<ThreadDataPack>& data, 
                    pipeline::BasicTask* parent) override;

    private:
        PositionTransform position_transform;   /*!< 位置变换器 */
        double comm_latency;                    /*!< 通信延迟（秒） */
        std::unique_ptr<LinearPredictor> predictor; /*!< 预测器 */
    };
}

#endif // PREDICT_PREDICT_SUBMODULE_H