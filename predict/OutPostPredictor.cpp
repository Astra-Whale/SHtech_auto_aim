//
// Created by Fuck-CV on 2024/5/14
//

#include "OutPostPredictor.hpp"

namespace predict
{
    /// 识别双阈值
    constexpr float conf_thres = 0.1f;

    void OutPostPredictor::predict(std::shared_ptr<ThreadDataPack>& data, PositionTransform& position_transform)
    {
        auto& detections = data->bboxes;
        auto q_raw = data->attitude.toQuaternion();
        auto& img = data->frame;
        auto t = data->time;
        auto& send = data->robotcommand;
        auto robot_status = data->robotstatus;

        Eigen::Quaternionf q(q_raw.matrix().transpose()); // 重建四元数
        Eigen::Matrix3d R_IW = q.matrix().cast<double>(); // 生成旋转矩阵
        position_transform.update_R_IW(R_IW);             // 更新旋转矩阵

        bbox_t armor;

        /// 过滤出敌方颜色的装甲板
        std::vector<bbox_t> new_detections; // new_detection: vector 是经过过滤后所有可能考虑的装甲板
        for (auto& d : detections)
        {
            // 因为识别问题，将所有敌方装甲板纳入考虑范围内，要求实际应用时视野内应只存在前哨站装甲板
            if ((int(robot_status.enemy_color) == d.color_id && d.tag_id != 0) ||
                (int(robot_status.enemy_color) == 2)) // 不能随意修改，否则会数组越界0-5
            {
                if (d.confidence >= conf_thres)
                    /* 阈值大于 conf_thres 直接放行 */
                    new_detections.push_back(d);
            }
        }

        bool need_init = false;

        if (new_detections.empty()) {
            // 连续丢失30帧以上确认目标丢失，重置滤波器
            if (lost_cnt >= 30) {
                send.distance = -1.f;
                send.yaw_speed = 0.f;
                need_init = true;
                lost_cnt = 0;
                return;
            }
            lost_cnt++;
        }

        if (need_init)
        {
            OutPostFilterMember m_opfm = position_transform.pnp_get_outpost_member(armor.pts, armor.tag_id);
            filter_x.reset(m_opfm.OutPostCenterWP(0, 0), t);
            filter_y.reset(m_opfm.OutPostCenterWP(1, 0), t);
            filter_angle.reset(m_opfm.ArmorAngle, t);                                                                           // 重置 x, y 轴, 角度滤波器

            m_opfm.OutPostCenterWP(2, 0) -= TrajectoryCompensation(m_opfm.OutPostCenterWP, robot_status.robot_speed_mps);       // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(m_opfm.OutPostCenterWP);                                                   // point camera: 目标在相机坐标系下的坐标
            double s_yaw_spd = 0;                                                                                               // s_yaw_spd: yaw轴速度计算值
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(m_opfm.OutPostCenterWP);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = (float)s_yaw_spd;
            send.pitch_angle = (float)s_pitch;
            send.pitch_speed = (float)0;
            send.shoot_mode = ShootMode::FOLLOW;
        }
        else {
            OutPostFilterMember m_opfm = position_transform.pnp_get_outpost_member(armor.pts, armor.tag_id);
            Eigen::Matrix<double, 1, 1> z_k_x{ m_opfm.OutPostCenterWP(0, 0) };                                                  // z_k_x: x轴滤波器观测量
            Eigen::Matrix<double, 1, 1> z_k_y{ m_opfm.OutPostCenterWP(1, 0) };                                                  // z_k_y: y轴滤波器观测量

            // 取角度 准备跳装甲板检测
            double armorAngle = m_opfm.ArmorAngle;
            auto filter_angle_last_state = filter_angle.get_state();
            double predictArmorAngle = filter_angle_last_state(0, 0) + filter_angle_last_state(1, 0) * filter_angle.get_duration(t);

            // 跳装甲板判断
            double delta = remainder(armorAngle - predictArmorAngle, 360);
            double delta_jump = delta;
            if (fabs(delta) > 120. / 2)
            {
                delta_jump = remainder(delta + 360, 120.);
            }
            armorAngle = predictArmorAngle + delta_jump;
            Eigen::Matrix<double, 1, 1> z_k_angle{ armorAngle };

            auto p_x = filter_x.update(z_k_x, t);                                                                               // p_x: x轴滤波器状态量
            auto p_y = filter_y.update(z_k_y, t);                                                                               // p_y: y轴滤波器状态量
            auto p_angle = filter_angle.update(z_k_angle, t);

            double ft = FlightTimePredictOutPost(Pos3D(p_x(0, 0), p_y(0, 0), m_opfm.OutPostCenterWP(2, 0)), robot_status.robot_speed_mps);      // ft: 预测弹丸飞行时间
            auto now_t = std::chrono::high_resolution_clock::now();
            double process_latency = duration_cast<microseconds>(now_t - t).count() / 1e6;
            double t_delay = ft + comm_latency + process_latency;

            Pos3D outpost_center_pw{ p_x(0, 0) + t_delay * p_x(1, 0), p_y(0, 0) + t_delay * p_y(1, 0), m_opfm.OutPostCenterWP(2, 0) };           // s_pw: t_delay后旋转中心预测点
            Pos3D outpost_Aim = outpost_center_pw - outpost_center_pw.normalized() * position_transform.outpost_radius;

            outpost_Aim(2, 0) -= TrajectoryCompensation(outpost_Aim, robot_status.robot_speed_mps);                             // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(outpost_Aim);                                                              // point camera: 目标在相机坐标系下的坐标
            double s_yaw_spd = 0;       // s_yaw_spd: yaw轴速度计算值
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(outpost_Aim);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = (float)s_yaw_spd;
            send.pitch_angle = (float)s_pitch;
            send.pitch_speed = (float)0;
            // 射击判据：
            double angle_at_t = p_angle(0, 0) + p_angle(1, 0) * t_delay;
            if (fabs(remainder(angle_at_t, 120)) < 0.5)
                send.shoot_mode = ShootMode::COMMON;
            else
                send.shoot_mode = ShootMode::FOLLOW;
        }
    }
};