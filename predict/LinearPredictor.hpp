//
// Created by Haoran-Jiang on 2021/11/25.
//

#ifndef PREDICT_LINEAR_PREDICTOR_H
#define PREDICT_LINEAR_PREDICTOR_H

// modules
#include "predict.hpp"
#include "common.hpp"
#include "kalman.h"

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
    class LinearPredictor
    {
    private:
        bool last_track{false}; // 上一次是否有追踪目标
        Pos3D last_pw;          // 上一次世界坐标
        bbox_t last_bbox;       // 上一次预测框
        using _filter = Kalman<1, 2>;
        using Matx1 = _filter::Matrix_x1d;
        using Matxx = _filter::Matrix_xxd;
        using Matxz = _filter::Matrix_xzd;
        using Matz1 = _filter::Matrix_z1d;
        using Matzx = _filter::Matrix_zxd;
        using Matzz = _filter::Matrix_zzd;
        _filter filter_x; // x轴滤波
        _filter filter_y; // y轴滤波

    public:
        explicit LinearPredictor()
        {
            last_track = false;
            ///初始化滤波器参数
            Matxx A = Matxx::Identity(); //转移矩阵
            Matzx H;                     //观测矩阵
            Matxx R;                     //过程噪声矩阵
            Matzz Q{0.1};                //测量噪声矩阵
            Matx1 init{0, 0};            //初始值
            ///初始化观测矩阵
            H(0, 0) = 1;
            ///初始化过程方差
            R(0, 0) = 0.01;
            for (int i = 1; i < 2; i++)
            {
                R(i, i) = 100;
            }
            ///初始化滤波器
            filter_x = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
            filter_y = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
        };
        void predict(std::shared_ptr<ThreadDataPack> &data, PositionTransform &position_transform);
    };
}

#endif // PREDICT_LINEAR_PREDICTOR_H