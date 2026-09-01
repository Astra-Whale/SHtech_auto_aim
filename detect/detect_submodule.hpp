//
// DetectSubModule
// Runs the selected inference backend.
//

#ifndef DETECT_DETECT_SUBMODULE_H
#define DETECT_DETECT_SUBMODULE_H

// submodules
#include "backend.hpp"

// modules
#include "common.hpp"

// packages
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

namespace detect
{
    struct DetectConfig : pipeline::ModuleConfig
    {
    };

    /**
     * @brief   检测子模块
     * @details 包装原有 Detect 逻辑为 SubModule，用于 PipelineTask
     */
    class DetectSubModule : public pipeline::SubModule
    {
    public:
        /**
         * @brief   构造函数
         * @param[in] OnnxFileName 用于推理的 Onnx 文件路径
         */
        DetectSubModule(const DetectConfig& config, const std::string& OnnxFileName);
        virtual ~DetectSubModule() = default;

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  SubModuleResult 表示当前数据包的处理结果
         */
        SubModuleResult process(std::shared_ptr<ThreadDataPack> data, 
                    const pipeline::BasicTask* parent) override;

    private:
        DetectConfig config_;
        std::unique_ptr<BackEnd> model; /*!< 推理模型指针 */
    };
}

#endif // DETECT_DETECT_SUBMODULE_H
