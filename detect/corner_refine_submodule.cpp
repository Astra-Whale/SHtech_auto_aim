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

        static std::string output_dir = "./frames/";

        if (_imgshow)
        {
            static const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {255, 255, 255}};
            cv::Mat im2show = data->frame.clone();
            for (const auto &b : data->bboxes)
            {
                cv::line(im2show, b.pts[0], b.pts[1], colors[2], 1);
                cv::line(im2show, b.pts[1], b.pts[2], colors[2], 1);
                cv::line(im2show, b.pts[2], b.pts[3], colors[2], 1);
                cv::line(im2show, b.pts[3], b.pts[0], colors[2], 1);

                const char source_tag = b.source == DetectionSource::TRADITIONAL ? 'T' : 'N';
                cv::putText(im2show,
                            std::string(1, source_tag),
                            b.pts[0],
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            colors[b.color_id]);
            }

            static bool dir_created = false;
            if (!dir_created)
            {
                system(("mkdir -p " + output_dir).c_str());
                dir_created = true;
            }

            int key = cv::waitKey(1);
            if (key == 'p' || key == 'P') {
                std::string filename = output_dir + "corner_refine_" + std::to_string(data->frame_counter) + ".png";
                cv::imwrite(filename, im2show);
            }

            cv::imshow("corner_refine_submodule", im2show);
            cv::waitKey(1);
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
