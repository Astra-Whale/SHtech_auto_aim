//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "predict.hpp"

namespace predict
{
    void Predict::operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter) const
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

        PredictorAdaptiveEKF predictor;
        bool last_mode_is_autoaim = true;

        do
        {
            auto obj = pipbefore.get(); /*!< 从上一线程的缓存队列获取报文指针 */
            cv::Mat img_show;
            predictor.predict(obj, img_show, _show);
            pipafter.put(obj); /*!< 向下一线程的缓存队列提交报文指针*/
            if (_debug)
            {
                LOGM_S("[predict] yaw: %.2f@%.2f, pitch: %.2f@%.2f", obj->robotcommand.yaw_angle, obj->robotcommand.yaw_speed, obj->robotcommand.pitch_angle, obj->robotcommand.pitch_speed);
            }
            if (_show)
            {
                cv::imshow("predict",img_show);cv::waitKey(1);
            }
        } while (_run);
        LOGM_S("[predict] stop");
    }
}
