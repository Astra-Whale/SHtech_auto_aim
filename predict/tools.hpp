//
// Created by Haoran-Jiang on 2021/11/20.
//

#ifndef PREDICT_TOOLS_H
#define PREDICT_TOOLS_H

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
    class PositionTransform
    {
    private:
        Eigen::Matrix3d R_CI;          // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
        Eigen::Matrix3d F;             // 相机内参矩阵EIGEN-Matrix
        Eigen::Matrix<double, 1, 5> C; // 相机畸变矩阵EIGEN-Matrix
        cv::Mat R_CI_MAT;              // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
        cv::Mat F_MAT;                 // 相机内参矩阵CV-Mat
        cv::Mat C_MAT;                 // 相机畸变矩阵CV-Mat
    public:
        explicit PositionTransform()
        {
            cv::FileStorage fin(PROJECT_DIR "/asset/camera-param.yml", cv::FileStorage::READ);
            fin["Tcb"] >> R_CI_MAT;
            fin["K"] >> F_MAT;
            fin["D"] >> C_MAT;
            cv::cv2eigen(R_CI_MAT, R_CI);
            cv::cv2eigen(F_MAT, F);
            cv::cv2eigen(C_MAT, C);
        }
        // pnp解算:获取相机坐标系内装甲板坐标
        Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], int armor_number)
        {

            static const std::vector<cv::Point3d> pw_small = {// 单位：m
                                                              {-0.066, 0.027, 0.},
                                                              {-0.066, -0.027, 0.},
                                                              {0.066, -0.027, 0.},
                                                              {0.066, 0.027, 0.}};
            static const std::vector<cv::Point3d> pw_big = {// 单位：m
                                                            {-0.115, 0.029, 0.},
                                                            {-0.115, -0.029, 0.},
                                                            {0.115, -0.029, 0.},
                                                            {0.115, 0.029, 0.}};
            std::vector<cv::Point2d> pu(p, p + 4);
            cv::Mat rvec, tvec;

            if (armor_number == 0 || armor_number == 1 || armor_number == 8)
                cv::solvePnP(pw_big, pu, F_MAT, C_MAT, rvec, tvec);
            else
                cv::solvePnP(pw_small, pu, F_MAT, C_MAT, rvec, tvec);

            Eigen::Vector3d pc;
            cv::cv2eigen(tvec, pc);
            pc[0] += 0.0278;
            pc[1] += 0.0165;
            pc[2] += 0.0659;
            return pc;
        }
        // 相机坐标系内坐标--->世界坐标系内坐标
        inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d &pc, const Eigen::Matrix3d &R_IW)
        {
            auto R_WC = (R_CI * R_IW).transpose();
            return R_WC * pc;
        }

        // 世界坐标系内坐标--->相机坐标系内坐标
        inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_IW)
        {
            auto R_CW = R_CI * R_IW;
            return R_CW * pw;
        }

        // 相机坐标系内坐标--->图像坐标系内像素坐标
        inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc)
        {
            return F * pc / pc(2, 0);
        }
    };
    static inline double TrajectoryCompensation(double distance, double pitch_angle, double shoot_speed = 15.)
    {
        double a = 9.8 * 9.8 * 0.25;
        double b = -shoot_speed * shoot_speed -
                   distance * 9.8 * cos(M_PI_2 + pitch_angle);
        double c = distance * distance;
        double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
        double fly_time = sqrt(t_2); // 子弹飞行时间（单位:s）
        // 解出抬枪高度，即子弹下坠高度
        double height = 0.5 * 9.8 * t_2;
        return height;
    }
}

#endif //PREDICT_TOOLS_H
