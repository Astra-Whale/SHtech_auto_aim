//
// Created for pipeline refactor - CornerRefineSubModule
// Moves armor corner refinement out of DetectSubModule
//

#include "corner_refine_submodule.hpp"

namespace detect
{
    CornerRefineSubModule::CornerRefineSubModule(const CornerRefineConfig& config)
        : SubModule(SubModuleName::CORNER_REFINE),
          config_(config),
          corner_optimizer(config.adjust_threshold)
    {
        if (config_.adjust_threshold) {
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

    SubModuleResult CornerRefineSubModule::process(std::shared_ptr<ThreadDataPack> data,
                                                   const pipeline::BasicTask* parent)
    {
        if (config_.adjust_threshold) {
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
        double total_roi_ms = 0.0;
        double total_preprocess_ms = 0.0;
        double total_find_light_ms = 0.0;
        double total_select_ms = 0.0;
        double total_final_check_ms = 0.0;
        double total_visualize_ms = 0.0;

        int armors_success = 0;
        int armors_failed = 0;
        int lights_attempted = static_cast<int>(data->bboxes.size()) * 2;
        int lights_success = 0;
        int candidate_light_bars = 0;

        for (auto& bbox : data->bboxes)
        {
            CornerRefineCallStats stats;
            const auto refined_corners =
                corner_optimizer.optimizeCorners(
                    data->frame,
                    bbox.pts,
                    config_.debug.show_image,
                    config_.debug.log_text ? &stats : nullptr);

            total_roi_ms += stats.roi_ms;
            total_preprocess_ms += stats.preprocess_ms;
            total_find_light_ms += stats.find_light_ms;
            total_select_ms += stats.select_ms;
            total_final_check_ms += stats.final_check_ms;
            total_visualize_ms += stats.visualize_ms;
            lights_success += stats.successful_lights;
            candidate_light_bars += stats.candidate_light_bars;

            if (refined_corners.has_value()) {
                for (int i = 0; i < 4; i++)
                {
                    bbox.pts[i] = (*refined_corners)[i];
                }

                bbox.source = DetectionSource::TRADITIONAL;
                armors_success++;
            } else {
                bbox.source = DetectionSource::NEURAL_NETWORK;
                armors_failed++;
            }
        }

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
                std::string filename = output_dir + "corner_refine_" + std::to_string(data->index) + ".png";
                cv::imwrite(filename, im2show);
            }

            cv::imshow("corner_refine_submodule", im2show);
            cv::waitKey(1);
        }

        auto t_opt_end = std::chrono::steady_clock::now();
        if (config_.debug.log_text)
        {
            const double frame_total_ms =
                std::chrono::duration_cast<std::chrono::duration<double>>(t_opt_end - t_opt_start).count() * 1000;
            const int lights_failed = lights_attempted - lights_success;
            const double armor_count = static_cast<double>(data->bboxes.size());

            LOGM_F("[corner_refine] Summary: armors=%zu success=%d fail=%d lights_attempted=%d lights_success=%d lights_fail=%d candidates=%d total=%.2lfms avg=%.2lfms",
                   data->bboxes.size(),
                   armors_success,
                   armors_failed,
                   lights_attempted,
                   lights_success,
                   lights_failed,
                   candidate_light_bars,
                   frame_total_ms,
                   armor_count > 0.0 ? frame_total_ms / armor_count : 0.0);
            LOGM_F("[corner_refine] Breakdown: roi=%.2lfms preprocess=%.2lfms find_light=%.2lfms select=%.2lfms final_check=%.2lfms visualize=%.2lfms",
                   total_roi_ms,
                   total_preprocess_ms,
                   total_find_light_ms,
                   total_select_ms,
                   total_final_check_ms,
                   total_visualize_ms);
        }

        return SubModuleResult::SUCCESS;
    }
}
