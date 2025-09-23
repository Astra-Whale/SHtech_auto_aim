//
// Created for pipeline refactor - PredictSubModule
// Wraps original Predict logic as SubModule
//

#include "predict_submodule.hpp"

namespace predict
{
    void PredictSubModule::init(const std::string camera_param, int latency)
    {
        LOGM_S("[predict_submodule] init");
        
        position_transform = PositionTransform(camera_param);
        comm_latency = latency / 1e3;
        
        // 创建预测器实例
        predictor = std::make_unique<LinearPredictor>(comm_latency);
        
        LOGM_S("[predict_submodule] ready");
        SubModule::init();
    }

    bool PredictSubModule::process(std::shared_ptr<ThreadDataPack>& data, 
                                   pipeline::BasicTask* parent)
    {
        if (!_init)
        {
            LOGE_S("[predict_submodule]Error: process before init.");
            return false;
        }

        auto t1 = std::chrono::steady_clock::now();
        
        // 执行预测
        predictor->predict(data, position_transform);
        
        auto t2 = std::chrono::steady_clock::now();

        // 调试信息
        if (_debug)
        {
            auto &send = data->robotcommand;
            LOGM_S("[predict_submodule] pitch %6.2f, yaw %6.2f, dist %4.1f",
                   send.pitch_angle, send.yaw_angle,
                   (float)send.distance / 10);
        }
        
        // 显示结果（如果需要）
        if (_show)
        {
            // 预测模块的显示逻辑（如果需要的话）
        }

        auto t3 = std::chrono::steady_clock::now();
        // LOGM_S(
        //     "PredictSubModule Predict %.2lfms Show %.2lfms", 
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000
        // );
        
        // 预测总是成功的，返回 true
        return true;
    }
}