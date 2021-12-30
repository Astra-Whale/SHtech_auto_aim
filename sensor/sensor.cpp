//
// Created by Haoran Jiang on 21-10-02
// Read sensor data from cammer/videofile/imu/A-board
//

#include "sensor.hpp"
namespace sensor
{
    float val_limit(float input, float max)
    {
        return input < max ? input > -max ? input : -max : max;
    }
    void Sensor::operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter)
    {
        /**
         * @brief 检查类是否正确初始化
         */
        if (!_init)
        {
            LOGE_S("[sensor]Error: run before init.");
            return;
        }

        LOGM_S("[sensor] running");

        if (imu != nullptr)
            imu->start();

        int totalFrameCounter = 0; /*!< 总帧数计数器 */
        fps_counter total_fps("total_fps");

        do
        {
            auto obj = pipbefore.get();           /*!< 从上一线程的缓存队列获取报文指针 */
            bool state = video->read(obj->frame); /*!< 读取是否成功 */
            obj->index = totalFrameCounter++;
            obj->time = std::chrono::high_resolution_clock::now();

            if (!state)
            {
                LOGE_S("[sensor]Error: read image fail!");
                LOGM_S("[sensor] Total frames handled: %d", totalFrameCounter);
                LOGM_S("[sensor] ReOpen Camera");
                video->close();
                video->init();
                pipbefore.put(obj);
                continue;
            }

            if (obj->frame.empty())
            {
                LOGW_S("empty image");
                pipbefore.put(obj);
                continue;
            }

            if (comm.isOpen() && obj->robotcommand.distance != -1)
            {
                auto &send = obj->robotcommand;
                auto &_attitude = obj->attitude;
                comm.transmit(_attitude.yaw + val_limit(send.yaw_angle, 10),
                              _attitude.pitch + val_limit(send.pitch_angle, 10),
                              send.yaw_speed, send.distance);
                if (_debug)
                {
                    LOGM_S("[transmit] p-p:%6.2f | p-m:%6.2f | p-s:%6.2f | y-p:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f",
                           send.pitch_angle, _attitude.pitch,
                           _attitude.pitch + val_limit(send.pitch_angle, 10),
                           send.yaw_angle, _attitude.yaw,
                           _attitude.yaw + val_limit(send.yaw_angle, 10),
                           send.yaw_speed);
                }
            }

            if (imu != nullptr)
                imu->get_attitude(obj->attitude);

            /**
             * @brief 当需要展示结果时，绘制 bounding box
             */
            if (_show)
            {
                /*cv::Mat im2show = obj->frame.clone();
                cv::imshow("sensor", im2show);
                cv::waitKey(1);*/
            }

            /**
             * @brief 当需要显示调试信息时，打印检测到的 bounding box 数量
             */
            if (_debug)
            {
                LOGM_S("[sensor]Info: Idx = %d, Bytes = %d", obj->index, obj->frame.size().height * obj->frame.size().width);
            }
            pipafter.put(obj); /*!< 向下一线程的缓存队列提交报文指针*/
        } while (_run);
        LOGM_S("[sensor] stop");
    }
}
