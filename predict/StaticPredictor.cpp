//
// Created by Haoran-Jiang on 2021/11/20.
//

#include "StaticPredictor.hpp"

namespace predict
{

    /// 选择高度限制
    constexpr double height_thres = -20.;
    /// 识别双阈值
    constexpr float conf_thres = 0.6f;
    /// 远距离弹量控制
    constexpr double distant_threshold = 6.;

    void StaticPredictor::predict(std::shared_ptr<ThreadDataPack> &data)
    {
        auto &detections = data->bboxes;
        auto q_raw = data->attitude.toQuaternion();
        auto &img = data->frame;
        auto t = data->time;
        auto &send = data->robotcommand;
        auto robot_status = data->robotstatus;

        Eigen::Quaternionf q(q_raw.matrix().transpose());
        Eigen::Matrix3d R_IW = q.matrix().cast<double>(); // 生成旋转矩阵

        bbox_t armor;

        /// 过滤出敌方颜色的装甲板 && 判断是否有英雄出现
        std::vector<bbox_t> new_detections; // new_detection: vector 是经过过滤后所有可能考虑的装甲板
        for (auto &d : detections)
        {
            if (int(robot_status.enemy_color) == d.color_id && d.tag_id != 0 && d.tag_id != 6) // 不能随意修改，否则会数组越界0-5
            {
                /* 放行正确颜色的装甲板 */
                Eigen::Vector3d m_pc = position_transform.pnp_get_pc(d.pts, d.tag_id); // point camera: 目标在相机坐标系下的坐标
                Eigen::Vector3d m_pw = position_transform.pc_to_pw(m_pc, R_IW);        // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
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
            send.distance = -1;
            //if (DEBUG) std::cout << "No valid armor!" << std::endl;
            return;
        }
        armor = new_detections.front();
        Eigen::Vector3d m_pc = position_transform.pnp_get_pc(armor.pts, armor.tag_id); // point camera: 目标在相机坐标系下的坐标
        Eigen::Vector3d m_pw = position_transform.pc_to_pw(m_pc, R_IW);                // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
        double mc_yaw = std::atan2(m_pc(1, 0), m_pc(0, 0));
        double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0)); // yaw的测量值，单位弧度
        double m_pitch = std::atan2(m_pw(2, 0),
                                    sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0))); // pitch的测量值，单位弧度
        double distance = m_pw.norm();
        double height = TrajectoryCompensation(distance, m_pitch);
        Eigen::Vector3d s_pw{m_pw(0, 0), m_pw(1, 0), m_pw(2, 0) - height}; // 抬枪后预测点

        Eigen::Vector3d s_pc = position_transform.pw_to_pc(s_pw, R_IW);
        double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
        double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

        send.distance = (int)(distance * 10);
        send.yaw_angle = (float)s_yaw;
        send.pitch_angle = (float)s_pitch;
    }
}