//
// Created by Haoran-Jiang on 2021/11/20.
//

#ifndef PREDICT_PREDICT_H
#define PREDICT_PREDICT_H

// submodules
#include "tools.hpp"
#include "StaticPredictor.hpp"
#include "LinearPredictor.hpp"

// modules
#include "common.hpp"

// packages
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    using pipline::BasicTask;

    /**
     * @brief   预测类
     */
    class Predict : public BasicTask
    {
    public:
        /**
         * @brief   传感器类初始化
         * @details 创建 PositionTransform 类实例用于位置解算
         * @param[in]
         */
        void init(const std::string camera_param, int latency = 20)
        {
            position_transform = PositionTransform(camera_param);
            comm_latency = latency / 1e3;
            BasicTask::init();
        }
        /**
         * @brief   预测线程主函数
         * @details 读取上一线程提交至缓存队列的报文指针 detection_obj_t*::obj
         *          调用 Predictor 进行预测并将结果写入报文 robotcommand 对象中
         *          将报文指针提交给下一线程的缓存队列
         * @param[in] pipbefore 与预测的上一流程交互的线程间通信对象
         * @param[in] pipafter  与预测的下一流程交互的线程间通信对象
         * @note    通过 stop() 控制启停
         *          必须先进行初始化
         */
        void operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter);

    private:
        PositionTransform position_transform;
        double comm_latency;
    };
}

#endif // PREDICT_PREDICT_H
