//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework: Refact framework
// Detect armor from opencv mat
//

//submodules
#include "TRTModule.hpp"
#include "detect.hpp"
#include "chrono"
#include <iostream>
namespace detect
{

    void Detect::init(const std::string OnnxFileName)
    {
        LOGM_S("[detect] init");
        model.reset(new TRTModule(OnnxFileName));
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
            auto obj = pipebefore.get(this);           /*!< 从上一线程的缓存队列获取报文指针 */
            if (obj == nullptr)
            {
                continue;
            }

            auto t2 = std::chrono::steady_clock::now();

            (*model)(obj->frame, obj->bboxes); /*!< 对报文中的图片 (frame) 进行推理并将结果存入报文 (bboxes)*/
            
            auto t3 = std::chrono::steady_clock::now();
            /**
             * @brief 当需要展示结果时，绘制 bounding box
             */
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
                }
                cv::imshow("sensor", im2show);
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
                    LOGM_S("[detect]Detect_Data: colorid: %d, tag_id: %d",b.color_id, b.tag_id);
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
        } while (_run);
        LOGM_S("[detect] stop");
    }
}
