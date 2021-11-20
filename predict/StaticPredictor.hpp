//
// Created by Haoran-Jiang on 2021/11/20.
//

#ifndef PREDICT_STATIC_PREDICTOR_H
#define PREDICT_STATIC_PREDICTOR_H

//modules
#include "predict.hpp"
#include "common.hpp"

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
    class StaticPredictor
    {
    private:
        PositionTransform position_transform;

    public:
        explicit StaticPredictor() : position_transform() {};
        void predict(std::shared_ptr<ThreadDataPack> &data);
    };
}

#endif //PREDICT_STATIC_PREDICTOR_H