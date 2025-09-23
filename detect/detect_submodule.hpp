//
// Created for pipeline refactor - DetectSubModule
// Wraps original Detect logic as SubModule
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
    /**
     * @brief   检测子模块
     * @details 包装原有 Detect 逻辑为 SubModule，用于 CompositeTask
     */
    class DetectSubModule : public pipeline::SubModule
    {
    public:
        DetectSubModule() : SubModule() {}
        virtual ~DetectSubModule() = default;

        /**
         * @brief   子模块初始化
         * @param[in] OnnxFileName 用于推理的 Onnx 文件路径
         */
        void init(const std::string OnnxFileName);

        /**
         * @brief   子模块处理函数
         * @param[in,out] data   输入输出数据包，直接在原数据上修改
         * @param[in] parent     父任务指针，用于生命周期检查
         * @return  bool         返回 true 表示数据应该传递到下游，false 表示丢弃数据
         */
        bool process(std::shared_ptr<ThreadDataPack>& data, 
                    pipeline::BasicTask* parent) override;

    private:
        std::unique_ptr<BackEnd> model; /*!< 推理模型指针 */
    };
}

#endif // DETECT_DETECT_SUBMODULE_H