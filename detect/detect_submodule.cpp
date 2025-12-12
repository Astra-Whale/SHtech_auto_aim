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
    DetectSubModule::DetectSubModule(const std::string& OnnxFileName, bool adjust_) 
    : SubModule(SubModuleName::DETECT),
      adjust(adjust_),
      corner_optimizer(adjust=adjust_)
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

        if (adjust) {
            // 创建窗口
            cv::namedWindow("detector trackbar", cv::WINDOW_AUTOSIZE);
        
            // 👇 创建滑动条
            cv::createTrackbar(
                "Binary Threshold",           // 滑动条名称
                "detector trackbar",        // 所属窗口名
                &binary_thres,      // 关联的整型变量（实时更新）
                255,                   // 最大值
                0      // 回调函数
            );
        }
        
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

        cv::Mat input_frame = data->frame;
        std::vector<bbox_t> output_bboxes;
        int halfx = input_frame.size[1] / 2;
        int halfy = input_frame.size[0] / 2;
        if (center)
        {
            input_frame = input_frame.rowRange(halfy - 100, halfy + 284).colRange(halfx - 320, halfx + 320); // crop size 384*640 in center
        }
        auto t2 = std::chrono::steady_clock::now();

        // 执行推理
        (*model)(input_frame, output_bboxes);

        auto t3 = std::chrono::steady_clock::now();

        if (center)
        {
            for (auto &t_bbox : output_bboxes)
            {
                for (int i = 0; i < 4; i++)
                {
                    t_bbox.pts[i].x += halfx - 320;
                    t_bbox.pts[i].y += halfy - 100;
                }
            }
        }
        data->bboxes = output_bboxes;

        if (adjust) {
            corner_optimizer.setBinaryThreshold(binary_thres);
        }
        else {
            if (data->robotstatus.enemy_color == EnemyColor::RED) {
                corner_optimizer.setBinaryThreshold(bin_threshold_for_red);
            }
            else if (data->robotstatus.enemy_color == EnemyColor::BLUE) {
                corner_optimizer.setBinaryThreshold(bin_threshold_for_blue);
            }
            else {
                corner_optimizer.setBinaryThreshold(bin_threshold_for_blue);
    
                LOGW_S("[detect] Warning: enemy color not set, using default binary threshold");
                LOGW_F("[detect] Warning: enemy color not set, using default binary threshold");
            }
        }

        std::vector<bbox_t> optimized_bboxes;
        // Corner optimization
        if (!data->bboxes.empty())
        {
            auto t_opt_start = std::chrono::steady_clock::now();

            // For each detected armor, optimize its corners
            for (auto &bbox : data->bboxes)
            {
                std::vector<cv::Point2f> yolo_corners;
                yolo_corners.resize(4);

                for (int i = 0; i < 4; i++)
                {
                    yolo_corners[i] = bbox.pts[i];
                }

                // Optimize corners using our ArmorCornerOptimizer
                std::vector<cv::Point2f> optimized_corners =
                    corner_optimizer.optimizeCorners(data->frame, bbox.pts, _imgshow);

                // if (optimized_corners.empty()) {

                //     continue;
                // }
                
                // // Update corners with optimized ones
                // for (int i = 0; i < 4; i++)
                // {
                //     bbox.pts[i] = optimized_corners[i];
                // }

                if (!optimized_corners.empty()) {
                    for (int i = 0; i < 4; i++)
                    {
                        bbox.pts[i] = optimized_corners[i];
                    }

                    optimized_bboxes.push_back(bbox);
                }
                else {
                    optimized_bboxes.push_back(bbox);
                }
            }

            data->bboxes = optimized_bboxes;

            // LOGT_S();
            // cout << data->bboxes.size() << endl;
            // if (data->bboxes.size() == 1) {
            //     cout << data->bboxes[0].tag_id << endl;
            //     cout << -1 << endl;
            // }
            // else {
            //     cout << data->bboxes[0].tag_id << endl;
            //     cout << data->bboxes[1].tag_id << endl;
            // }

            auto t_opt_end = std::chrono::steady_clock::now(); 
            if (_debugprint)
            {
                LOGM_S("[detect]Info: corner optimization took %.2lfms",
                        std::chrono::duration_cast<std::chrono::duration<double>>(t_opt_end - t_opt_start).count() * 1000);
            }
        }

        /**
         * @brief 当需要展示结果时，绘制 bounding box
         */
        static int frame_counter = 0;
        static std::string output_dir = "./frames/"; // 图片保存目录
        
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
                // if (center)
                //     cv::putText(im2show, "center", b.pts[2], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
                // else
                //     cv::putText(im2show, "all", b.pts[2], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
            }

            // 创建目录（只需执行一次）
            static bool dir_created = false;
            if (!dir_created)
            {
                system(("mkdir -p " + output_dir).c_str()); // Linux/macOS
                // 如果是Windows系统改用：
                // system(("mkdir " + output_dir).c_str());
                dir_created = true;
            }

            int key = cv::waitKey(1);
            // // 保存当前帧（按顺序编号）
            if (key == 'p' || key == 'P') {
                std::string filename = output_dir + "frame_" + std::to_string(frame_counter++) + ".png";
                cv::imwrite(filename, im2show);
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
        
        auto t4 = std::chrono::steady_clock::now();
        if(_debugprint)
            LOGM_S(
                "DetectSubModule Inference %.2lfms Show %.2lfms", 
                std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000,
                std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()*1000
            );

        if (output_bboxes.size() == 0)
        {
            center = !center;
        }
        
        // 检测总是成功的，返回 true
        return SubModuleResult::SUCCESS;
    }
}