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
            auto t1 = std::chrono::steady_clock::now();
            auto obj = pipebefore.get(this);           /*!< 从上一线程的缓存队列获取报文指针 */
            if (obj == nullptr)
            {
                continue;
            }
            cv::Mat img_show;

            auto t2 = std::chrono::steady_clock::now();
            _predictor.predict(obj, position_transform);
            auto t3 = std::chrono::steady_clock::now();

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
            auto t4 = std::chrono::steady_clock::now();
            // LOGM_S(
            //     "Predict Read %.2lfms Detect %.2lfms Put %.2lfms", 
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000,
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()*1000
            // );
        } while (_run);
        LOGM_S("[predict] stop");
    }
}
