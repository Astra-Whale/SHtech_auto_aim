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
    static inline void map_bbox_to_source(const DetectInputMap &map, bbox_t &bbox)
    {
        if (!map.valid || map.dst_size.width <= 0 || map.dst_size.height <= 0)
        {
            return;
        }

        const float scale_x = static_cast<float>(map.src_roi.width) / static_cast<float>(map.dst_size.width);
        const float scale_y = static_cast<float>(map.src_roi.height) / static_cast<float>(map.dst_size.height);

        for (auto &pt : bbox.pts)
        {
            pt.x = map.src_roi.x + pt.x * scale_x;
            pt.y = map.src_roi.y + pt.y * scale_y;
        }
    }

    DetectSubModule::DetectSubModule(const DetectConfig& config, const std::string& OnnxFileName)
        : SubModule(SubModuleName::DETECT), config_(config)
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

    SubModuleResult DetectSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                             const pipeline::BasicTask* parent)
    {
        std::vector<bbox_t> output_bboxes;
        data->bboxes.clear();
        auto t2 = std::chrono::steady_clock::now();

        if (!data->detect_input_map.valid || data->detect_input.empty())
        {
            LOGE_S("[detect] missing preprocess output");
            return SubModuleResult::FAILURE;
        }

        (*model)(data->detect_input, output_bboxes);

        auto t3 = std::chrono::steady_clock::now();

        for (auto &t_bbox : output_bboxes)
        {
            map_bbox_to_source(data->detect_input_map, t_bbox);
        }
        data->bboxes = output_bboxes;

        static int frame_counter = 0;
        static std::string output_dir = "./frames/";

        if (config_.debug.show_image)
        {
            static const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {255, 255, 255}};
            cv::Mat im2show = data->frame.clone();
            for (const auto &b : data->bboxes)
            {
                cv::line(im2show, b.pts[0], b.pts[1], colors[2], 1);
                cv::line(im2show, b.pts[1], b.pts[2], colors[2], 1);
                cv::line(im2show, b.pts[2], b.pts[3], colors[2], 1);
                cv::line(im2show, b.pts[3], b.pts[0], colors[2], 1);
                cv::putText(im2show, "N", b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
            }

            static bool dir_created = false;
            if (!dir_created)
            {
                system(("mkdir -p " + output_dir).c_str());
                dir_created = true;
            }

            int key = cv::waitKey(1);
            if (key == 'p' || key == 'P') {
                std::string filename = output_dir + "frame_" + std::to_string(frame_counter++) + ".png";
                cv::imwrite(filename, im2show);
            }

            cv::imshow("detect_submodule", im2show);
            cv::waitKey(1);
        }

        if (config_.debug.log_text)
        {
            LOGM_S("[detect] Info: detected %ld objects", data->bboxes.size());
            for (const auto &b : data->bboxes)
            {
                LOGM_S("[detect] Detect_Data: colorid: %d, tag_id: %d", b.color_id, b.tag_id);
            }
        }

        auto t4 = std::chrono::steady_clock::now();
        if (config_.debug.log_text)
            LOGM_S(
                "DetectSubModule Inference %.2lfms Show %.2lfms",
                std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count() * 1000,
                std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count() * 1000
            );

        return SubModuleResult::SUCCESS;
    }
}
