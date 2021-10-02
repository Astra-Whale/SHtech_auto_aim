//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "predict.hpp"

namespace predict
{
    void Predict::operator()(autoaim_pipline &pipafter, autoaim_pipline &pipbefore) const
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

        //PredictorAdaptiveEKF predictor;
        bool last_mode_is_autoaim = true;
        //auto t1 = system_clock::now();

        do
        {
            auto obj = pipbefore.get(); /*!< 从上一线程的缓存队列获取报文指针 */
            /*if (obj->robotstatus->program_mode == ProgramMode::AUTO_AIM)
            {
                auto detections = detections_sub.pop_for(50);
                RobotCmd robot_cmd;

                if (!last_mode_is_autoaim)
                    predictor.clear();
                last_mode_is_autoaim = true;

                bool ok = predictor.predict(detections, robot_cmd, im2show);

                if (!ok)
                {
                    throw timeout_error;
                }

                robot_cmd_pub.push(robot_cmd);
            }
            else if (obj->robotstatus->program_mode == ProgramMode::SMALL_ENERGY || obj->robotstatus->program_mode == ProgramMode::BIG_ENERGY)
            {
            }
            else
            {
                LOGE_S("[predict]Error: Program_mode %u not implemented.", (unsigned int)obj->robotstatus->program_mode);
            }*/
            pipafter.put(obj); /*!< 向下一线程的缓存队列提交报文指针*/
        } while (_run);
        LOGM_S("[predict] stop");
    }
}
