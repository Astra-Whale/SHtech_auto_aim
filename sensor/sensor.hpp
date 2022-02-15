//
// Created by Haoran Jiang on 21-10-02
// Read sensor data from cammer/videofile/imu/A-board
//

#ifndef SENSOR_SENSOR_H
#define SENSOR_SENSOR_H

//submodules
#include "cam_wrapper.hpp"
#include "imu.hpp"

//modules
#include "common.hpp"
#include "comm.hpp"

//packages
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

namespace sensor
{
    using pipline::BasicTask;

    /**
     * @brief   传感器类
     */
    class Sensor : public BasicTask
    {
    public:
        /**
         * @brief   传感器类初始化
         * @details 创建 Warpper_Head 子类实例用于获取图像
         * @param[in] 
         */

        ~Sensor()
        {
            stop();
            video->close();
            imu->close();
        }

        void init(const std::string VideoSource, const std::string ImuSource, const std::string port, const std::string flip_image)
        {
            LOGM_S("[sensor] comm I/O on %s", port.c_str());
            if (port != "None")
            {
                comm.open(port);
            }
            LOGM_S("[sensor] video input from %s", VideoSource.c_str());
            if (VideoSource == "0")
            {
                video = new HikCamWrapper();
            }
            else
            {
                video = new VideoWrapper(VideoSource);
            }
            if (!video->init())
            {
                LOGE_S("[sensor]Error: Initialize video stream failed");
            }
            LOGM_S("[senosr] video ready");
            LOGM_S("[senosr] IMU input from %s", ImuSource.c_str());
            imu = nullptr;
            if (ImuSource == "UART")
            {
                if (comm.isOpen() == false)
                {
                    LOGE_S("[sensor] port not specified while UartImu Enabled");
                }
                else
                    imu = new UartIMU(&comm);
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

        /**
         * @brief   装甲板检测线程主函数
         * @details 读取上一线程提交至缓存队列的报文指针 detection_obj_t*::obj
         *          调用 model 指向的 TensorRT 推理器对报文中的 cv::Mat::frame 变量进行推理
         *          将推理结果写入报文中的 std::vector<bbox_t>::bboxes 变量
         *          将报文指针提交给下一线程的缓存队列
         * @param[in] pipbefore 与装甲板检测的上一流程交互的线程间通信对象
         * @param[in] pipafter  与装甲板检测的下一流程交互的线程间通信对象
         * @note    通过 stop() 控制启停
         *          必须先进行初始化
         */
        void operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter);

    private:
        WrapperHead *video; /*!< unique_ptr 智能指针 指向一个用于 TensorRT 推理的 TRTModule 对象 在 init 期间完成初始化 */
        ImuHead *imu;
        Comm comm;
        bool is_image_input_flipped;
    };
}

#endif //SENSOR_SENSOR_H
