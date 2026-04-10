//
// Created for pipeline refactor - CornerRefineSubModule
// Moves armor corner refinement out of DetectSubModule
//

#include "corner_refine_submodule.hpp"

namespace detect
{
    CornerRefineSubModule::CornerRefineSubModule(bool adjust_)
        : SubModule(SubModuleName::CORNER_REFINE),
          adjust(adjust_),
          corner_optimizer(adjust_)
    {
        if (adjust) {
            cv::namedWindow("detector trackbar", cv::WINDOW_AUTOSIZE);
            cv::createTrackbar(
                "Binary Threshold",
                "detector trackbar",
                &binary_thres,
                255,
                0
            );
        }

        LOGM_S("[corner_refine] construction completed");
    }

    bool CornerRefineSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const
    {
        return data->submodule_results[static_cast<uint8_t>(SubModuleName::DETECT)] != SubModuleResult::SUCCESS;
    }

    SubModuleResult CornerRefineSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                                   const pipeline::BasicTask* parent)
    {
        if (adjust) {
            corner_optimizer.setBinaryThreshold(binary_thres);
        } else {
            if (data->robotstatus.enemy_color == EnemyColor::RED) {
                corner_optimizer.setBinaryThreshold(bin_threshold_for_red);
            } else if (data->robotstatus.enemy_color == EnemyColor::BLUE) {
                corner_optimizer.setBinaryThreshold(bin_threshold_for_blue);
            } else {
                corner_optimizer.setBinaryThreshold(bin_threshold_for_blue);
                LOGW_S("[corner_refine] Warning: enemy color not set, using default binary threshold");
                LOGW_F("[corner_refine] Warning: enemy color not set, using default binary threshold");
            }
        }

        if (data->bboxes.empty())
        {
            return SubModuleResult::SUCCESS;
        }

        auto t_opt_start = std::chrono::steady_clock::now();

        for (auto& bbox : data->bboxes)
        {
            const auto refined_corners =
                corner_optimizer.optimizeCorners(data->frame, bbox.pts, _imgshow);

            if (refined_corners.has_value()) {
                for (int i = 0; i < 4; i++)
                {
                    bbox.pts[i] = (*refined_corners)[i];
                }

                bbox.source = DetectionSource::TRADITIONAL;
            } else {
                bbox.source = DetectionSource::NEURAL_NETWORK;
            }
        }

        auto t_opt_end = std::chrono::steady_clock::now();
        if (_debugprint)
        {
            LOGM_S("[corner_refine] Info: corner refinement took %.2lfms",
                   std::chrono::duration_cast<std::chrono::duration<double>>(t_opt_end - t_opt_start).count() * 1000);
        }

        return SubModuleResult::SUCCESS;
    }
}
