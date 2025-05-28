#include "detect.hpp"
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

    /**
     * @brief   装甲板检测初始化
     * @details 创建 TRTModule 实例用于 TensorRT 推理
     * @param[in] OnnxFileName 用于推理的 Onnx 文件路径
     */
    void Detect::init(const std::string OnnxFileName)
    {
        LOGM_S("[detect] init");
#if INFERENCE_BACKEND_TYPE == 1
        model.reset(new ONNX(OnnxFileName));
#elif INFERENCE_BACKEND_TYPE == 2
        model.reset(new TRTModule(OnnxFileName));
#elif INFERENCE_BACKEND_TYPE == 3
        model.reset(new AXCL(OnnxFileName));
#else
#error "Invalid INFERENCE_BACKEND_TYPE"
#endif
        // model.reset(new BackEnd(OnnxFileName));
        LOGM_S("[detect] ready");
        BasicTask::init();
    }

    /**
     * @brief   装甲板检测线程主函数
     * @details 读取上一线程提交至缓存队列的报文指针 detection_obj_t*::obj
     *          调用 model 指向的 TensorRT 推理器对报文中的 cv::Mat::frame 变量进行推理
     *          将推理结果写入报文中的 std::vector<bbox_t>::bboxes 变量
     *          将报文指针提交给下一线程的缓存队列
     * @param[in] pipebefore 与装甲板检测的上一流程交互的线程间通信对象
     * @param[in] pipeafter  与装甲板检测的下一流程交互的线程间通信对象
     * @note    通过 stop() 控制启停
     *          必须先进行初始化
     */

    void Detect::operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter)
    {
        /**
         * @brief 检查类是否正确初始化
         */
        if (!_init)
        {
            LOGE_S("[detect]Error: run before init.");
            return;
        }
        LOGM_S("[detect] running");
        do
        {
            auto t1 = std::chrono::steady_clock::now();
            auto obj = pipebefore.get(this); /*!< 从上一线程的缓存队列获取报文指针 */
            if (obj == nullptr)
            {
                continue;
            }
            cv::Mat input_frame = obj->frame;
            std::vector<bbox_t> output_bboxes;
            int halfx = input_frame.size[1] / 2;
            int halfy = input_frame.size[0] / 2;
            if (center)
            {
                input_frame = input_frame.rowRange(halfy - 100, halfy + 284).colRange(halfx - 320, halfx + 320); // crop size 384*640 in center
            }
            auto t2 = std::chrono::steady_clock::now();
            (*model)(input_frame, output_bboxes); /*!< 对报文中的图片 (frame) 进行推理并将结果存入报文 (bboxes)*/

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
            obj->bboxes = output_bboxes;

            // Corner optimization
            if (!obj->bboxes.empty())
            {
                auto t_opt_start = std::chrono::steady_clock::now();

                // For each detected armor, optimize its corners
                for (auto &bbox : obj->bboxes)
                {
                    std::vector<cv::Point2f> yolo_corners;
                    yolo_corners.resize(4);

                    for (int i = 0; i < 4; i++)
                    {
                        yolo_corners[i] = bbox.pts[i];
                    }

                    // Optimize corners using our ArmorCornerOptimizer
                    std::vector<cv::Point2f> optimized_corners =
                        corner_optimizer.optimizeCorners(obj->frame, bbox.pts, _show);

                    if (optimized_corners.empty()) {
                        continue;
                    }
                    
                    // Update corners with optimized ones
                    for (int i = 0; i < 4; i++)
                    {
                        bbox.pts[i] = optimized_corners[i];
                    }
                }

                auto t_opt_end = std::chrono::steady_clock::now();
                if (_debug)
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

            if (_show)
            {
                static const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {255, 255, 255}};
                cv::Mat im2show = obj->frame.clone();
                for (const auto &b : obj->bboxes)
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

                // 保存当前帧（按顺序编号）
                std::string filename = output_dir + "frame_" + std::to_string(frame_counter++) + ".png";
                cv::imwrite(filename, im2show);
                cv::imshow("detect", im2show);
                cv::waitKey(1);
            }

            /**
             * @brief 当需要显示调试信息时，打印检测到的 bounding box 数量
             */
            if (_debug)
            {
                LOGM_S("[detect]Info: detected %ld objects", obj->bboxes.size());
                for (const auto &b : obj->bboxes)
                {
                    LOGM_S("[detect]Detect_Data: colorid: %d, tag_id: %d", b.color_id, b.tag_id);
                }
            }
            pipeafter.put(obj, this); /*!< 向下一线程的缓存队列提交报文指针*/
            auto t4 = std::chrono::steady_clock::now();
            // LOGM_S(
            //     "Detect Read %.2lfms Detect %.2lfms Put %.2lfms",
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000,
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()*1000
            // );

            if (output_bboxes.size() == 0)
            {
                center = !center;
            }
        } while (_run);
        LOGM_S("[detect] stop");
    }
}