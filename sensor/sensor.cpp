//
// Created by Haoran Jiang on 21-10-02
// Read sensor data from cammer/videofile/imu/A-board
//

#include "sensor.hpp"

//submodules
#include <video/video_wrapper.hpp>
#ifdef ENABLE_HIKCAM
#warning ENABLE_HIKCAM 
#include <hikcam/hikcam_wrapper.hpp>
#else
#warning 
#warning DISABLE_HIKCAM 
#endif

namespace sensor
{
    /**
     * @brief 控制输入的最大范围
     *
     * @param input 输入量
     * @param max 最大值（绝对值）
     * @return float 约化的输出值
     */
    float val_limit(float input, float max)
    {
        return input < max ? input > -max ? input : -max : max;
    }

    /**
     * @brief 用于输出角度滤波的辅助类
     *
     */
    class AngleFilter
    {
    private:
        float angle{0.f};
        bool init{true};

    public:
        /**
         * @brief 重置滤波器
         *
         */
        void reset()
        {
            angle = 0.f;
            init = true;
        }

        /**
         * @brief 更新滤波器输入
         *
         * @param input 输入
         * @return float 输出
         */
        float update(float input)
        {
            if (init)
            {
                angle = input;
                init = false;
            }
            else
            {
                angle = angle * 0.9f + input * 0.1f;
            }
            return angle;
        }

        /**
         * @brief 获取滤波器输出
         *
         * @return float 输出
         */
        float output()
        {
            return angle;
        }
    };
    void Sensor::init(const std::string VideoSource, const std::string ImuSource, const std::string port, const std::string flip_image)
        {
            LOGM_S("[sensor] comm I/O on %s", port.c_str());
            LOGM_S("[sensor] video input from %s", VideoSource.c_str());
            if (VideoSource == "0")
            {
                #ifdef ENABLE_HIKCAM
                    video = new HikCamWrapper();
                #else
                    LOGE_S("[sensor] hikcam not enabled!");
                #endif
            }
            else
            {
                video = new VideoWrapper(VideoSource);
            }
            while (!video->init())
            {
                LOGE_S("[sensor]Error: Initialize video stream failed");
            }
            LOGM_S("[senosr] video ready");
            LOGM_S("[senosr] IMU input from %s", ImuSource.c_str());
            imu = nullptr;
            if (ImuSource == "UART")
            {
                imu = new UartIMU(port);
            }
            if (imu == nullptr || !imu->init())
            {
                LOGE_S("[sensor]Error: IMU init failed");
            }
            if (flip_image == "1")
            {
                is_image_input_flipped = true;
				LOGW_S("[sensor] Input image will be flipped");
            }
            else 
            {
                is_image_input_flipped = false;
				LOGW_S("[sensor] Input image will not be flipped");
            }
            LOGM_S("[senosr] IMU ready");
            LOGM_S("[sensor] ready");
            BasicTask::init();
        }
        
        
        
    void Sensor::operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter)
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
        AngleFilter pitch_angle_filter;

        do
        {
            auto t1 = std::chrono::steady_clock::now();
            auto obj = pipebefore.get(this);           /*!< 从上一线程的缓存队列获取报文指针 */
            if (obj == nullptr)
            {
            	LOGM_S("[sensor] null ptr get");
                continue;
            }
            bool state = video->read(obj->frame, _debug); /*!< 读取是否成功 */
            obj->index = totalFrameCounter++;
            obj->time = std::chrono::high_resolution_clock::now();

            if (!state)
            {
                if (_debug)
                {
                    LOGE_S("[sensor]Error: read image fail!");
                    LOGM_S("[sensor] Total frames handled: %d", totalFrameCounter);
                    LOGM_S("[sensor] ReOpen Camera");
                }
                video->close( _debug);
                video->init(_debug);
                pipebefore.put(obj);
                continue;
            }

            if (obj->frame.empty())
            {
                LOGW_S("empty image");
                pipebefore.put(obj);
                continue;
            }

            if (is_image_input_flipped)
            {
                cv::flip(obj->frame, obj->frame, -1);
            }
            auto t2 = std::chrono::steady_clock::now();

            if (imu != nullptr && imu->is_open())
            {
                auto &send = obj->robotcommand;
                auto &_attitude = obj->attitude;
                imu->transmit_cmd(
                    _attitude.yaw + val_limit(send.yaw_angle, 10),
                    pitch_angle_filter.update(_attitude.pitch + val_limit(send.pitch_angle, 10)),
                    send.yaw_speed, 
                    send.pitch_speed, 
                    send.distance,
                    static_cast<uint8_t>(send.shoot_mode == ShootMode::COMMON)
                );
            }
            else
            {
                pitch_angle_filter.reset();
            }
                
            if (_debug)
            {
                auto &send = obj->robotcommand;
                auto &_attitude = obj->attitude;
                LOGM_S("[transmit] p-p:%6.2f | p-m:%6.2f | p-s:%6.2f | y-p:%6.2f | y-m:%6.2f | y-s:%6.2f | ys-s:%6.2f",
                        send.pitch_angle, _attitude.pitch,
                        pitch_angle_filter.output(),
                        send.yaw_angle, _attitude.yaw,
                        _attitude.yaw + val_limit(send.yaw_angle, 10),
                        send.yaw_speed);
            }

            if (imu != nullptr)
            {
                imu->get_attitude(obj->attitude);
                imu->get_robotstatus(obj->robotstatus);
            }
            auto t3 = std::chrono::steady_clock::now();

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
                CNT_FPS(total_fps, {});
                // LOGM_S("[sensor]Info: Idx = %d, Bytes = %d", obj->index, obj->frame.size().height * obj->frame.size().width);
            }
            pipeafter.put(obj, this); /*!< 向下一线程的缓存队列提交报文指针*/
            auto t4 = std::chrono::steady_clock::now();
            // LOGM_S(
            //     "Sensor Read %.2lfms Detect %.2lfms Put %.2lfms", 
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000,
            //     std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count()*1000
            // );
        } while (_run);
        LOGM_S("[sensor] stop");
    }
}
