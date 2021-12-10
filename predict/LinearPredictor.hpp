//
// Created by Haoran-Jiang on 2021/11/25.
//

#ifndef PREDICT_LINEAR_PREDICTOR_H
#define PREDICT_LINEAR_PREDICTOR_H

//modules
#include "predict.hpp"
#include "common.hpp"
#include "kalman.h"

//packages
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    class LinearPredictor
    {
    private:
        PositionTransform position_transform{};
        bool last_track{false};
        Pos3D last_pw;
        bbox_t last_bbox;
        using _filter = Kalman<1, 2>;
        using Matx1 = _filter::Matrix_x1d;
        using Matxx = _filter::Matrix_xxd;
        using Matxz = _filter::Matrix_xzd;
        using Matz1 = _filter::Matrix_z1d;
        using Matzx = _filter::Matrix_zxd;
        using Matzz = _filter::Matrix_zzd;
        _filter filter_x;
        _filter filter_y;

    public:
        explicit LinearPredictor()
        {
            last_track = false;
            // 初始化滤波器参数
            Matxx A = Matxx::Identity();
            Matzx H;
            H(0, 0) = 1;
            Matxx R;
            R(0, 0) = 0.01;
            for (int i = 1; i < 2; i++)
            {
                R(i, i) = 100;
            }
            Matzz Q{0.1};
            Matx1 init{0, 0};
            filter_x = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
            filter_y = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
        };
        void predict(std::shared_ptr<ThreadDataPack> &data);
    };
}

#endif //PREDICT_LINEAR_PREDICTOR_H