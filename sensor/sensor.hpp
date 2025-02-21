//
// Created by Haoran Jiang on 21-10-02
// Read sensor data from cammer/videofile/imu/A-board
//

#ifndef SENSOR_SENSOR_H
#define SENSOR_SENSOR_H

//submodules
#include "cam_wrapper.hpp"
#include "UartIMU/uartimu.hpp"

//modules
#include "common.hpp"

//packages
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

namespace sensor
{
    using pipeline::BasicTask;
    using pipeline::autoaim_pipeline;

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

        void init(const std::string VideoSource, const std::string ImuSource, const std::string port, const std::string flip_image);

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
        void operator()(autoaim_pipeline &pipebefore, autoaim_pipeline &pipeafter);

    private:
        WrapperHead *video; /*!< unique_ptr 智能指针 指向一个用于 TensorRT 推理的 TRTModule 对象 在 init 期间完成初始化 */
        UartIMU *imu;
        bool is_image_input_flipped;
    };
}

#endif //SENSOR_SENSOR_H
