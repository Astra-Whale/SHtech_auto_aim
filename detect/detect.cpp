//
// Inherit from SJTU-CV-2021/autoaim/autoaim.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework: Refact framework
// Detect armor from opencv mat
//

#include "detect.hpp"
namespace detect
{
    void Detect::operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter)
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
            auto obj = pipbefore.get();        /*!< 从上一线程的缓存队列获取报文指针*/
            (*model)(obj->frame, obj->bboxes); /*!< 对报文中的图片 (frame) 进行推理并将结果存入报文 (bboxes)*/

            /**
             * @brief 当需要展示结果时，绘制 bounding box
             */
            if (_show)
            {
                static const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
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
            pipafter.put(obj); /*!< 向下一线程的缓存队列提交报文指针*/
        } while (_run);
        LOGM_S("[detect] stop");
    }
}
