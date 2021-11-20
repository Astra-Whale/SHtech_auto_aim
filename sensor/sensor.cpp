//
// Created by Haoran Jiang on 21-10-02
// Read sensor data from cammer/videofile/imu/A-board
//

#include "sensor.hpp"
namespace sensor
{
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

            if (comm.isOpen() && (obj->robotcommand.pitch_angle > 0.05f || obj -> robotcommand.pitch_angle < -0.05f))
	    {
                float pitch_delta = obj->robotcommand.pitch_angle;
		pitch_delta = (pitch_delta > +5 ? +5 : pitch_delta);
		pitch_delta = (pitch_delta < -5 ? -5 : pitch_delta);
		LOGM_S("[sensor]given %6.2f, now %6.2f, set %6.2f, spd %6.2f", obj->robotcommand.pitch_angle, obj->attitude.pitch, obj->attitude.pitch + pitch_delta, obj->robotcommand.pitch_speed);
		float yaw_delta = obj->robotcommand.yaw_angle * 0.8f;
		yaw_delta = (yaw_delta > +5 ? +5 : yaw_delta);
		yaw_delta = (yaw_delta < -5 ? -5 : yaw_delta);
                //comm.transmit(obj->attitude.yaw + yaw_delta, obj->attitude.pitch + pitch_delta, 2);
	    }

            if (imu != nullptr)
	    {
                imu->get_quaternion(obj->quaternion);
		imu->get_attitude(obj->attitude);
	    }

            if (obj->frame.empty())
            {
                LOGW_S("empty image");
                pipbefore.put(obj);
                continue;
            }

	    //CNT_FPS(total_fps,{});

            /**
             * @brief 当需要展示结果时，绘制 bounding box
             */
            if (_show)
            {
                cv::Mat im2show = obj->frame.clone();
                cv::imshow("sensor", im2show);
                cv::waitKey(1);
            }

            /**
             * @brief 当需要显示调试信息时，打印检测到的 bounding box 数量
             */
            if (_debug)
            {
                LOGM_S("[sensor]Info: Idx = %d, yaw %.1f, pitch %.1f, yaw_set %6.2f", obj->index, obj->attitude.yaw, obj->attitude.pitch, obj->robotcommand.yaw_angle);
            }
            pipafter.put(obj); /*!< 向下一线程的缓存队列提交报文指针*/
        } while (_run);
        LOGM_S("[sensor] stop");
    }
}
