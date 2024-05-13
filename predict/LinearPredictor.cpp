//
// Created by Haoran-Jiang on 2021/11/25.
//

#include "LinearPredictor.hpp"

namespace predict
{

    /// 选择高度限制
    constexpr double height_thres = -20.;
    /// 识别双阈值
    constexpr float conf_thres = 0.1f;
    /// 远距离弹量控制
    constexpr double distant_threshold = 6.;

    void LinearPredictor::predict(std::shared_ptr<ThreadDataPack> &data, PositionTransform &position_transform)
    {
        auto &detections = data->bboxes;
        auto q_raw = data->attitude.toQuaternion();
        auto &img = data->frame;
        auto t = data->time;
        auto &send = data->robotcommand;
        auto robot_status = data->robotstatus;

        Eigen::Quaternionf q(q_raw.matrix().transpose()); // 重建四元数
        Eigen::Matrix3d R_IW = q.matrix().cast<double>(); // 生成旋转矩阵
        position_transform.update_R_IW(R_IW);             // 更新旋转矩阵

        bbox_t armor;

        /// 过滤出敌方颜色的装甲板 && 判断是否有英雄出现
        std::vector<bbox_t> new_detections; // new_detection: vector 是经过过滤后所有可能考虑的装甲板
        for (auto &d : detections)
        {
            if ((int(robot_status.enemy_color) == d.color_id && d.tag_id != 0 && d.tag_id != 6) ||
                (int(robot_status.enemy_color) == 2) || true) // 不能随意修改，否则会数组越界0-5
            {
                /* 放行正确颜色的装甲板 */
                Pos3D m_pc = position_transform.pnp_get_pc(d.pts, d.tag_id); // point camera: 目标在相机坐标系下的坐标
                Pos3D m_pw = position_transform.pc_to_pw(m_pc);              // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                if (m_pw[2] < height_thres)
                {
                    LOGW_S("To High! height is %lf", m_pw[2]);
                    continue;
                }
                if (int(robot_status.game_state) == 0)
                {
                    double distance = m_pw.norm();
                    if (distance > distant_threshold)
                    {
                        LOGW_S("To Far! Distance is %lf", distance);
                        continue;
                    }
                }

                if (d.confidence >= conf_thres)
                    /* 阈值大于 conf_thres 直接放行 */
                    new_detections.push_back(d);
            }
        }
        if (new_detections.empty())
        {
            send.distance = -1.f;
            send.yaw_speed = 0.f;
            last_track = false;
            return;
        }

        bool selected = false;
        bool same_armor = false;
        bool same_id = false;
        bool need_init = false;

        if (last_track) // 寻找上一次打击的装甲板
        {
            // LOGM_S("[linear] Try Armor");
            for (auto &d : new_detections)
            {
                auto center = points_center(d.pts);
                Pos3D pw = position_transform.pnp_get_pw(d.pts, d.tag_id);
                if (last_track && (center.inside(get_ROI(last_bbox)) || is_same_armor_by_distance(last_pw, pw)))
                {
                    armor = d;
                    selected = true;
                    same_armor = true;
                    same_id = false;
                    need_init = false;
                    break;
                }
            }
        }

        if (!selected && last_track) // 寻找与上次同编号的装甲板
        {
            LOGM_S("[linear] Try ID");
            for (auto &d : new_detections)
            {
                if (d.tag_id == last_bbox.tag_id)
                {
                    LOGM_S("[Linear] Same ID");
                    armor = d;
                    selected = true;
                    same_armor = false;
                    same_id = true;
                    need_init = true;
                    break;
                }
            }
        }

        if (!selected) // 寻找最大的装甲板
        {
            double max_size = 0;
            for (auto &d : new_detections)
            {
                auto size = get_bbox_size(d);
                if (size > max_size)
                {
                    armor = d;
                    max_size = size;
                }
            }
            LOGM_S("[Linear] Sort by size");
            selected = true;
            same_armor = false;
            same_id = false;
            need_init = true;
        }
        
        if (same_armor)
        {
            Pos3D m_pw = position_transform.pnp_get_pw(armor.pts, armor.tag_id);                      // point world: 目标在世界坐标系下的坐标
            Eigen::Matrix<double, 1, 1> z_k_x{m_pw(0, 0)};                                            // z_k_x: x轴滤波器观测量
            Eigen::Matrix<double, 1, 1> z_k_y{m_pw(1, 0)};                                            // z_k_y: y轴滤波器观测量

            double m_yaw = position_transform.pnp_get_armor_angle(armor.pts, armor.tag_id);
            Eigen::Matrix<double, 1, 1> z_k_yaw{m_yaw};

            auto p_x = filter_x.update(z_k_x, t);                                                     // p_x: x轴滤波器状态量
            auto p_y = filter_y.update(z_k_y, t);                                                     // p_y: y轴滤波器状态量
            auto p_yaw = filter_yaw.update(z_k_yaw, t);

            // Switching strategy
            double armar_yaw_angle = p_yaw(0, 0);
            double armar_yaw_spd = p_yaw(1, 0);

            if (false)
            {
                send.shoot_mode = ShootMode::FOLLOW;
            } else {
                send.shoot_mode = ShootMode::COMMON;
            }
            if (false) {
                send.distance = -1.f;
                send.yaw_speed = 0.f;
                last_track = false;
            }

            double ft = FlightTimePredict(m_pw, robot_status.robot_speed_mps);                        // ft: 预测弹丸飞行时间
            auto now_t = std::chrono::high_resolution_clock::now();                                   //
            double process_latency = duration_cast<microseconds>(now_t - t).count() / 1e6;            //
            double t_delay = ft + comm_latency + process_latency;                                     //
            Pos3D s_pw{p_x(0, 0) + t_delay * p_x(1, 0), p_y(0, 0) + t_delay * p_y(1, 0), m_pw(2, 0)}; // s_pw: ft后预测点
            Eigen::Vector2d r_vec(p_x(0, 0), p_y(0, 0));                                              // 目标装甲板位矢
            Eigen::Vector2d v_vec(p_y(1, 0), -p_x(1, 0));                                             // 目标装甲板速度
            s_pw(2, 0) -= TrajectoryCompensation(s_pw, robot_status.robot_speed_mps);                 // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(s_pw);                                           // point camera: 目标在相机坐标系下的坐标
            double s_yaw_spd = -(r_vec.dot(v_vec)) / (r_vec.norm() * r_vec.norm()) / M_PI * 180.;     // s_yaw_spd: yaw轴速度计算值
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(s_pw);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = (float)s_yaw_spd;
            send.pitch_angle = (float)s_pitch;
        }
        else
        {
            Pos3D m_pw = position_transform.pnp_get_pw(armor.pts, armor.tag_id);        // point world: 目标在世界坐标系下的坐标
            filter_x.reset(m_pw(0, 0), t), filter_y.reset(m_pw(1, 0), t);               // 重置 x,y 轴滤波器
            double m_yaw = position_transform.pnp_get_armor_angle(armor.pts, armor.tag_id);
            filter_yaw.reset(m_yaw, t);
            double height = TrajectoryCompensation(m_pw, robot_status.robot_speed_mps); // height: 弹道下坠高度
            Pos3D s_pw{m_pw(0, 0), m_pw(1, 0), m_pw(2, 0) - height};                    // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(s_pw);                             // point camera: 目标在相机坐标系下的坐标
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(s_pw);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = 0.f;
            send.pitch_angle = (float)s_pitch;

            LOGW_S("[Linear] New Filter");
        }

        last_track = true;
        last_bbox = armor;
        last_pw = position_transform.pnp_get_pw(armor.pts, armor.tag_id);
    }
}
