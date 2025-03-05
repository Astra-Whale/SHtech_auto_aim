//
// Created by Fuck-CV on 2024/5/14
//

#ifndef PREDICT_OUTPOST_PREDICTOR_H
#define PREDICT_OUTPOST_PREDICTOR_H

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
    class OutPostPredictor
    {
    private:
        bool last_track{ false };       // whether last time has track target
        Pos3D last_pw;                  // last time position under world axis
        bbox_t last_bbox;               // last time predict box 
        double comm_latency;            // communication latency
        uint lost_cnt{ 0 };                  // lost counter
        using _filter = Kalman<1, 2>;
        using Matx1 = _filter::Matrix_x1d;
        using Matxx = _filter::Matrix_xxd;
        using Matxz = _filter::Matrix_xzd;
        using Matz1 = _filter::Matrix_z1d;
        using Matzx = _filter::Matrix_zxd;
        using Matzz = _filter::Matrix_zzd;
        _filter filter_x;               // x-axis filter
        _filter filter_y;               // y-axis filter
        _filter filter_angle;           // angle filter
    public:
        explicit OutPostPredictor(double latency = .020)
        {
            last_track = false;
            comm_latency = latency;

            // Initialization
            Matxx A = Matxx::Identity();
            Matzx H;
            Matxx R;
            Matzz Q{ 0.05 };
            Matx1 init{ 0, 0 };

            H(0, 0) = 1;
            R(0, 0) = 10;
            R(1, 1) = 10;

            filter_x = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
            filter_y = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());

            Q(0, 0) = 0.05;
            H(0, 0) = 1;
            R(0, 0) = 10;
            R(1, 1) = 10;
            filter_angle = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());

        };
        void predict(std::shared_ptr<ThreadDataPack>& data, PositionTransform& position_transform);
    };
}

#endif // PREDICT_OUTPOST_PREDICTOR_H