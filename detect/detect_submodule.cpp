//
// Created for pipeline refactor - DetectSubModule  
// Wraps original Detect logic as SubModule
//

#include "detect_submodule.hpp"
#include "chrono"
#include <iostream>

#if INFERENCE_BACKEND_TYPE == 1
#include "ONNX/ONNX.hpp"
#elif INFERENCE_BACKEND_TYPE == 2
#include "TensorRT/TRTModule.hpp"
#elif INFERENCE_BACKEND_TYPE == 3
#include "AXCL/AXCL.hpp"
#endif

namespace detect
{
    DetectSubModule::DetectSubModule(const std::string& OnnxFileName) : SubModule(SubModuleName::DETECT)
    {
        LOGM_S("[detect] constructing with model: %s", OnnxFileName.c_str());
        #if INFERENCE_BACKEND_TYPE == 1
            model.reset(new ONNX(OnnxFileName));
        #elif INFERENCE_BACKEND_TYPE == 2
            model.reset(new TRTModule(OnnxFileName));
        #elif INFERENCE_BACKEND_TYPE == 3
            model.reset(new AXCL(OnnxFileName));
        #else
            #error "Invalid INFERENCE_BACKEND_TYPE"
        #endif
        
        LOGM_S("[detect] model loaded");
    }

    bool DetectSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const
    {
        if(data->submodule_results[static_cast<uint8_t>(SubModuleName::SENSOR)] != SubModuleResult::SUCCESS)
            return true;
        return false;
    }

    SubModuleResult DetectSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                  const pipeline::BasicTask* parent)
    {
        auto t1 = std::chrono::steady_clock::now();

        // 执行推理
        (*model)(data->frame, data->bboxes);
        
        auto t2 = std::chrono::steady_clock::now();
        
        // 显示结果（如果需要）
        if (_imgshow)
        {
            static const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {255, 255, 255}};
            cv::Mat im2show = data->frame.clone();
            for (const auto &b : data->bboxes)
            {
                cv::line(im2show, b.pts[0], b.pts[1], colors[2], 2);
                cv::line(im2show, b.pts[1], b.pts[2], colors[2], 2);
                cv::line(im2show, b.pts[2], b.pts[3], colors[2], 2);
                cv::line(im2show, b.pts[3], b.pts[0], colors[2], 2);
                cv::putText(im2show, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
            }
            cv::imshow("detect_submodule", im2show);
            cv::waitKey(1);
        }

        // 调试信息
        if (_debugprint)
        {
            LOGM_S("[detect] Info: detected %ld objects", data->bboxes.size());
            for (const auto &b : data->bboxes)
            {
                LOGM_S("[detect] Detect_Data: colorid: %d, tag_id: %d", b.color_id, b.tag_id);
            }
        }
        
        auto t3 = std::chrono::steady_clock::now();
        if(_debugprint)
            LOGM_S(
                "[Detect] Inference %.2lfms Show %.2lfms", 
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
                std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000
            );
        
        // 检测总是成功的，返回 true
        return SubModuleResult::SUCCESS;
    }
}