//
// Created for pipeline refactor - main header for submodules
// Include all submodules for CompositeTask usage
//

#ifndef MAIN_SUBMODULES_H
#define MAIN_SUBMODULES_H

// 包含所有子模块
#include "sensor/sensor_submodule.hpp"
#include "detect/detect_submodule.hpp"
#include "predict/LinearPredictorSubModule.hpp"

// 包含通用头文件
#include "main.hpp"

namespace pipeline_test
{
    using namespace pipeline;
    
    /**
     * @brief   传感器复合任务
     * @details 包装 SensorSubModule 的 CompositeTask
     */
    class SensorCompositeTask : public CompositeTask
    {
    public:
        SensorCompositeTask() = default;
        
        void init(const std::string VideoSource, const std::string ImuSource, 
                  const std::string port, const std::string flip_image)
        {
            // 创建并注册 SensorSubModule（在构造时初始化设备）
            auto sensor_module = std::make_unique<sensor::SensorSubModule>(VideoSource, ImuSource, port, flip_image);
            sensor_module->init();  // 只启动设备和设置初始化标志
            register_submodule(std::move(sensor_module));
            
            CompositeTask::init();
        }
    };
    
    /**
     * @brief   检测复合任务
     * @details 包装 DetectSubModule 的 CompositeTask
     */
    class DetectCompositeTask : public CompositeTask
    {
    public:
        DetectCompositeTask() = default;
        
        void init(const std::string OnnxFileName)
        {
            // 创建并注册 DetectSubModule（在构造时初始化模型）
            auto detect_module = std::make_unique<detect::DetectSubModule>(OnnxFileName);
            detect_module->init();  // 只设置初始化标志
            register_submodule(std::move(detect_module));
            
            CompositeTask::init();
        }
    };
    
    /**
     * @brief   预测复合任务
     * @details 包装 LinearPredictorSubModule 的 CompositeTask
     */
    class PredictCompositeTask : public CompositeTask
    {
    public:
        PredictCompositeTask() = default;
        
        void init(const std::string camera_param, int latency = 20)
        {
            // 创建并注册 LinearPredictorSubModule（在构造时初始化）
            auto predict_module = std::make_unique<predict::LinearPredictorSubModule>(camera_param, latency);
            predict_module->init();  // 只设置初始化标志
            register_submodule(std::move(predict_module));
            
            CompositeTask::init();
        }
    };
}

#endif // MAIN_SUBMODULES_H