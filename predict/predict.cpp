//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "predict.hpp"

namespace predict
{
    void Predict::operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter)
    {
        /**
         * @brief 检查类是否正确初始化
         */
        if (!_init)
        {
            LOGE_S("[predict]Error: run before init.");
            return;
        }

        LOGM_S("[predict] running");

        LinearPredictor _predictor(comm_latency);

        do
        {
            auto obj = pipebefore.get(this);           /*!< 从上一线程的缓存队列获取报文指针 */
            if (obj == nullptr)
            {
                continue;
            }
            cv::Mat img_show;

            _predictor.predict(obj, position_transform);

            if (_debug)
            {
                auto &send = obj->robotcommand;
                LOGM_S("[predict] pitch %6.2f, yaw %6.2f, dist %4.1f",
                       send.pitch_angle, send.yaw_angle,
                       (float)send.distance / 10);
            }
            if (_show)
            {
            }

            pipeafter.put(obj, this); /*!< 向下一线程的缓存队列提交报文指针*/
        } while (_run);
        LOGM_S("[predict] stop");
    }
}
