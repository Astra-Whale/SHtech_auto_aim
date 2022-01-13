//
// Created by Qingcheng-Zhao on 2021/11/20.
//

#ifndef PREDICT_RVEC_PREDICTOR_H
#define PREDICT_RVEC_PREDICTOR_H

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
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    class RvecPredictor
    {
    private:
    public:
        explicit RvecPredictor() {}
        void predict(std::shared_ptr<ThreadDataPack> &data, PositionTransform &position_transform);
    };
}

#endif //PREDICT_RVEC_PREDICTOR_H
