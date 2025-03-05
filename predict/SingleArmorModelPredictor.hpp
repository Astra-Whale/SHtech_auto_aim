//
// Created by Haoran-Jiang on 2021/11/25.
//

#ifndef PREDICT_SINGLEARMOR_PREDICTOR_H
#define PREDICT_SINGLEARMOR_PREDICTOR_H

// modules
#include "predict.hpp"
#include "common.hpp"
#include "extendedKalmanFilter.h"

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

//暂时只考虑编号1到5的目标
#define TOTAL_TARGET_ACCOUNT 1

namespace predict
{
    class SingleArmorModelPredictor
    {
    private:
        bool last_track; // 上一次是否有追踪目标
        Pos3D last_pw;          // 上一次世界坐标
        uint lost_cnt;  //至今已有多久无法观测到某编号装甲板
        double another_r; //目前不被观测的一对装甲板的r
        bbox_t last_bbox;       // 上一次预测框
        double comm_latency;    // 通讯相关延迟
        //观察:装甲板1的x,y,yaw,共3项
        //状态:车辆中心的x,y,yaw,vx,vy,vyaw,装甲板距离车辆中心的距离r,共7项
        using _filter = ExtendedKalman<3, 7>;  //从泛型ekf类中创建一个三维观察七维状态的ekf
        using Matx1 = _filter::Matrix_x1d;
        using Matxx = _filter::Matrix_xxd;
        using Matxz = _filter::Matrix_xzd;
        using Matz1 = _filter::Matrix_z1d;
        using Matzx = _filter::Matrix_zxd;
        using Matzz = _filter::Matrix_zzd;
        _filter filter_target; // 为每辆车生成一个滤波器

        //单装甲板车中心预测器的状态函数，观测函数以及它们的雅可比矩阵
        static Matx1 state_transition(const Matx1 &x, double dt)
        {
            Matx1 x_new;
            x_new(0) = x(0) + x(3) * dt; // x = x + vx * dt
            x_new(1) = x(1) + x(4) * dt; // y = y + vy * dt
            x_new(2) = x(2) + x(5) * dt; // yaw = yaw + vyaw * dt
            x_new(3) = x(3);             // vx remains the same
            x_new(4) = x(4);             // vy remains the same
            x_new(5) = x(5);             // vyaw remains the same
            x_new(6) = x(6);             // r remains the same
            return x_new;
        }

        static Matxx state_transition_jacobian(const Matx1 &x, double dt)
        {
            Matxx F;
            F <<1, 0, 0,dt, 0, 0, 0,
                0, 1, 0, 0,dt, 0, 0,
                0, 0, 1, 0, 0,dt, 0,
                0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 1;
            return F;
        }

        /**
         * @brief 计算装甲板位置的观测方程
         * @details 坐标系定义：
         *   - 世界坐标系：地面为xy平面，z轴向下
         *   - 角度定义：x轴正半轴为0度，逆时针为正，范围[0, 360)度。与一般坐标系相同。可参看ArmorPalcementPC.get_angle()
         */
        static Matz1 observation_function(const Matx1 &x)
        {
            Matz1 z;
            z(0) = x(0) + x(6) * cos(x(2));  // armor_x = x + r*cos(yaw)
            z(1) = x(1) + x(6) * sin(x(2));  // armor_y = y + r*sin(yaw)
            z(2) = x(2);  //yaw is yaw
            return z;
        }

        static Matzx observation_jacobian(const Matx1 &x)
        {
            Matzx H;
            H <<1, 0, -x(6)*sin(x(2)), 0, 0, 0, cos(x(2)),
                0, 1, x(6)*cos(x(2)) , 0, 0, 0, sin(x(2)),
                0, 0, 1,               0, 0, 0, 0;
            return H;
        }

        // 利用当前维护的车体和装甲板角度，外加存储的另一组装甲板r，推算左侧装甲板对应的状态变量。为观测跳装甲板做准备。
        // 左侧，指的是从相机出发看的左侧，即：当前装甲板角度+90度。
        // 这里更改的是x_k1，即，需要先调用本函数，再调用update。
        // 如果即将跳跃到的装甲板组，我们之前已经观测到过，那么，我们利用之前观测得到的another_r预测它的位置。
        // 如果即将跳跃到的装甲板组是全新的，那么，我们使用默认的another_r来预测它的位置，目前它被初始化为0.25m。
        void turn_left_armor()
        {
            Matx1 state_vec = filter_target.get_state();
            // 只需current_state要改动yaw，交换r和another_r
            state_vec(2,0) += 90;
            std::swap(state_vec(6,0),another_r);
            filter_target.directly_change_state(state_vec);
        }

        // 同理turn_left_armor，只是装甲板变化为-90度。
        void turn_right_armor()
        {
            Matx1 state_vec = filter_target.get_state();
            // 只需current_state要改动yaw，交换r和another_r
            state_vec(2,0) -= 90;
            std::swap(state_vec(6,0),another_r);
            filter_target.directly_change_state(state_vec);
        }

        Eigen::Vector2d get_armor_velocity(const Matx1 &state)
        {
            // 装甲板速度 = 车心平移速度 + 旋转产生的速度
            double armor_vx = state(3) - state(6) * state(5) * sin(state(2));  // vx - r * ω * sin(θ)
            double armor_vy = state(4) + state(6) * state(5) * cos(state(2));  // vy + r * ω * cos(θ)
            return Eigen::Vector2d(armor_vx, armor_vy);
        }

    public:
        explicit SingleArmorModelPredictor(double latency = .020)
        {
            last_track= false;
            lost_cnt= 0;
            another_r=0.25;
            comm_latency = latency;
            ///定义并初始化过程方差, 观测方差, 初始值
            Matxx R;                     //过程噪声矩阵
            Matzz Q;               //测量噪声矩阵
            Matx1 init;            //初始值
            // 过程噪声矩阵设置
            R.setIdentity();
            R(0,0) = R(1,1) = 0.05;  // xy位置，使用sigma2_q_xy
            R(2,2) = 5.0;               // yaw，使用sigma2_q_yaw
            R(6,6) = 80.0;              // r，使用sigma2_q_r
            // 速度分量相关的过程噪声可以设置为位置的2-3倍
            R(3,3) = R(4,4) = 0.1;  // vx,vy,vz
            R(5,5) = R(2,2) * 2.0;      // vyaw

            // 观测噪声矩阵设置
            Q.setIdentity();
            Q(0,0) = Q(1,1 )= 4e-4;  // xy位置，使用r_xyz_factor
            Q(2,2) = 5e-3;              // yaw，使用r_yaw

            // 初始状态设置
            init.setZero();
            init(6) = 0.26;             // 初始半径0.26m
            
            ///初始化滤波器
            filter_target= _filter(
                R, Q, init, std::chrono::high_resolution_clock::now(),
                std::function<Matx1(const Matx1&, double)>(state_transition),
                std::function<Matz1(const Matx1&)>(observation_function),
                std::function<Matxx(const Matx1&, double)>(state_transition_jacobian),
                std::function<Matzx(const Matx1&)>(observation_jacobian));
            
        };
        void predict(std::shared_ptr<ThreadDataPack> &data, PositionTransform &position_transform);
    };
}

#endif // PREDICT_SINGLEARMOR_PREDICTOR_H