//
// Created for pipeline refactor - PreprocessSubModule
// Splits common detector image preprocessing out of DetectSubModule
//

#include "preprocess_submodule.hpp"

namespace detect
{
    PreprocessSubModule::PreprocessSubModule(const PreprocessConfig& config)
        : SubModule(SubModuleName::PREPROCESS), config_(config)
    {
        LOGM_S("[preprocess] construction completed");
    }

    SubModuleResult PreprocessSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                                 const pipeline::BasicTask* parent)
    {
        if (data->frame.empty())
        {
            data->detect_input.release();
            data->detect_input_map = {};
            LOGE_S("[preprocess] empty frame");
            return SubModuleResult::FAILURE;
        }

        resized_bgr_.create(INPUT_H, INPUT_W, CV_8UC3);
        data->detect_input.create(INPUT_H, INPUT_W, CV_8UC3);

        cv::resize(data->frame, resized_bgr_, cv::Size(INPUT_W, INPUT_H));
        cv::cvtColor(resized_bgr_, data->detect_input, cv::COLOR_BGR2RGB);

        data->detect_input_map.src_roi = cv::Rect(0, 0, data->frame.cols, data->frame.rows);
        data->detect_input_map.dst_size = cv::Size(INPUT_W, INPUT_H);
        data->detect_input_map.valid = true;

        if (config_.debug.show_image)
        {
            cv::imshow("detect_preprocess", data->detect_input);
            cv::waitKey(1);
        }

        if (config_.debug.log_text)
        {
            // LOGM_S("[preprocess] output %dx%d from source %dx%d",
            //        data->detect_input.cols,
            //        data->detect_input.rows,
            //        data->frame.cols,
            //        data->frame.rows);
        }

        return SubModuleResult::SUCCESS;
    }
}
