//
// Created by Harry-hhj on 2021/5/4.
//

#ifndef PREDICT_PREDICT_H
#define PREDICT_PREDICT_H

//submodules
//#include "PredictorAdaptiveEKF.h"
//#include "PredictorKalman.h"

//modules
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
    using pipline::BasicTask;
    using pipline::detection_obj_t;

    class Predict : public BasicTask
    {
    public:
        void operator()(autoaim_pipline &beforedet, autoaim_pipline &afterdet) const;

    private:
    };
}

#endif //PREDICT_PREDICT_H
