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

    using Pos3D = Eigen::Vector3d;

    class PositionTransform
    {
    private:
        Eigen::Matrix3d R_CI;          // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
        Eigen::Matrix3d F;             // 相机内参矩阵EIGEN-Matrix
        Eigen::Matrix<double, 1, 5> C; // 相机畸变矩阵EIGEN-Matrix
        Eigen::Matrix3d R_IW;          // 陀螺仪坐标系到世界坐标系旋转矩阵EIGEN-Matrix
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
        Pos3D pnp_get_pc(const cv::Point2f p[4], int armor_number)
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

            Pos3D pc;
            cv::cv2eigen(tvec, pc);
            pc[0] += 0.0278;
            pc[1] += 0.0165;
            pc[2] += 0.0659;
            return pc;
        }

        // 相机坐标系内坐标--->世界坐标系内坐标
        inline Pos3D pc_to_pw(const Pos3D &pc)
        {
            auto R_WC = (R_CI * R_IW).transpose();
            return R_WC * pc;
        }

        // 世界坐标系内坐标--->相机坐标系内坐标
        inline Pos3D pw_to_pc(const Pos3D &pw)
        {
            auto R_CW = R_CI * R_IW;
            return R_CW * pw;
        }

        // 相机坐标系内坐标--->图像坐标系内像素坐标
        inline Pos3D pc_to_pu(const Pos3D &pc)
        {
            return F * pc / pc(2, 0);
        }

        // pnp解算:获取世界坐标系内装甲板坐标
        inline Pos3D pnp_get_pw(const cv::Point2f p[4], int armor_number)
        {
            Pos3D pc = pnp_get_pc(p, armor_number);
            return pc_to_pw(pc);
        }

        // 更新世界坐标系--->相机坐标系旋转矩阵
        inline void update_R_IW(const Eigen::Matrix3d &_R_IW)
        {
            R_IW = _R_IW;
        }

        double pnp_get_angle(const cv::Point2f p[4], int armor_number)
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
            std::vector<cv::Point3d> pw_cur;
            std::vector<cv::Point2d> pu(p, p + 4);

            cv::Mat rvec, tvec, mat_R;
            Eigen::MatrixXd R, T, w_p[4];
            Pos3D e_x, a_hat;

            if (armor_number == 0 || armor_number == 1 || armor_number == 8)
                pw_cur = pw_big;
            else
                pw_cur = pw_small;
            cv::solvePnP(pw_cur, pu, F_MAT, C_MAT, rvec, tvec);

            cv::Rodrigues(rvec, mat_R);
            cv::cv2eigen(mat_R, R);
            cv::cv2eigen(tvec, T);

            for (int i = 0; i < 4; ++i) {
                cv::cv2eigen(pw_cur[i], w_p[i]);
                w_p[i] = R * w_p[i].transpose() + T;
            }

            e_x << 1, 0, 0;
            a_hat = w_p[2] - w_P[1];
            double angle = a_hat.dot(e_x) / a_hat.norm() / e_x.norm();
            std::cout << acos(angle) << std::endl;

            return angle;
        }
    };

    // xy平面上的两点距离
    static inline double distance_2D(const Pos3D &m_pw)
    {
        return sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0));
    }

    // 世界坐标系下计算pitch轴角度
    static inline double pw_to_pitch(const Pos3D &m_pw)
    {
        return std::atan2(m_pw(2, 0), distance_2D(m_pw));
    }

    // 世界坐标系下计算yaw轴角度
    static inline double pw_to_yaw(const Pos3D &m_pw)
    {
        return std::atan2(m_pw(1, 0), m_pw(0, 0));
    }

    //飞行时间预测
    static inline double FlightTimePredict(const Pos3D &pw, double shoot_speed = 15.)
    {
        double distance = distance_2D(pw);
        double pitch_angle = pw_to_pitch(pw);
        double a = 9.8 * 9.8 * 0.25;
        double b = -shoot_speed * shoot_speed -
                   distance * 9.8 * cos(M_PI_2 + pitch_angle);
        double c = distance * distance;
        double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
        double fly_time = sqrt(t_2); // 子弹飞行时间（单位:s）
        return fly_time;
    }

    //弹道补偿
    static inline double TrajectoryCompensation(const Pos3D &pw, double shoot_speed = 15.)
    {
        double distance = distance_2D(pw);
        double pitch_angle = pw_to_pitch(pw);
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

    // 计算任意四边形的中心
    static inline cv::Point2f points_center(cv::Point2f pts[4])
    {
        cv::Point2f center;
        center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;
        center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4;
        return center;
    }

    // 四点转化为矩形
    static inline cv::Rect2f get_ROI(bbox_t &armor, float coefficient = 1.0f)
    {
        auto center = points_center(armor.pts);
        auto w = std::max({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x}) -
                 std::min({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x});
        auto h = std::max({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y}) -
                 std::min({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y});
        return cv::Rect2f(center.x - w / 2, center.y - h / 2, w * coefficient, h * coefficient);
    }

    // 根据距离判断是否是同一块装甲板
    static inline bool is_same_armor_by_distance(const Pos3D old_m_pw, const Pos3D new_m_pw, const double distance_threshold = 0.15)
    {
        auto m_pw_delta = new_m_pw - old_m_pw;
        double distance = m_pw_delta.norm();
        if (distance < distance_threshold)
            return true;
        else
            return false;
    }

    //四边形面积
    static inline double get_bbox_size(const bbox_t &bx)
    {
        auto bx_a = sqrt(pow(bx.pts[0].x - bx.pts[1].x, 2) + pow(bx.pts[0].y - bx.pts[1].y, 2));
        auto bx_b = sqrt(pow(bx.pts[1].x - bx.pts[2].x, 2) + pow(bx.pts[1].y - bx.pts[2].y, 2));
        auto bx_c = sqrt(pow(bx.pts[2].x - bx.pts[3].x, 2) + pow(bx.pts[2].y - bx.pts[3].y, 2));
        auto bx_d = sqrt(pow(bx.pts[3].x - bx.pts[0].x, 2) + pow(bx.pts[3].y - bx.pts[0].y, 2));
        auto bx_z = (bx_a + bx_b + bx_c + bx_d) / 2;
        auto bx_size = 2 * sqrt((bx_z - bx_a) * (bx_z - bx_b) * (bx_z - bx_c) * (bx_z - bx_d));
        return bx_size;
    }

}

#endif //PREDICT_TOOLS_H
