//
// Created by Fuck-CV on 2024/5/14
//

#include "SingleArmorModelPredictor.hpp"

namespace predict
{
    /// 识别双阈值
    constexpr float conf_thres = 0.1f;

    void SingleArmorModelPredictor::predict(std::shared_ptr<ThreadDataPack>& data, PositionTransform& position_transform)
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
        Pos3D speed_pc,aim_pc;

        /// 过滤出敌方颜色的装甲板
        std::vector<bbox_t> new_detections; // new_detection: vector 是经过过滤后所有可能考虑的装甲板
        for (auto& d : detections)
        {
            // 因为识别问题，将所有敌方装甲板纳入考虑范围内，要求实际应用时视野内应只存在前哨站装甲板
            if ((int(robot_status.enemy_color) == d.color_id && d.tag_id != 0) ||
                (int(robot_status.enemy_color) == 2)) //只有在调试模式下，我们才会
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
        else if (need_init)
        {
            sort(new_detections.begin(), new_detections.end(), 
                [](const bbox_t &a, const bbox_t &b) {
                    return a.pts[0].x > b.pts[0].x;  // 从大到小排序
                }
            ); //从右到左排序，临时措施
            armor = new_detections[0];//xy:我似乎需要先选择装甲板？这里直接选择视野里的第一块
            //将本帧获得的装甲板作为初始状态送入滤波器，将滤波器各自参数初始化
            ArmorPlacementPW armor_placement = position_transform.pnp_get_ArmorPlacementPW(armor.pts, armor.tag_id);
            Matx1 fliter_status;
            double r = 0.25;//暂时假设装甲板到车中心的距离是0.25。这个估计是一个经验数值。它只影响滤波器初始化时的准确度。
            fliter_status <<    armor_placement.x() + r *cos(armor_placement.get_angle()),
                                armor_placement.y() + r *sin(armor_placement.get_angle()),
                                armor_placement.get_angle(),
                                0.0,
                                0.0,
                                0.0,
                                r;
            filter_target.reset(fliter_status,t);


            armor_placement.location(2, 0) -= TrajectoryCompensation(armor_placement.location, robot_status.robot_speed_mps);       // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(armor_placement.location);                                                   // point camera: 目标在相机坐标系下的坐标                                                                                               // s_yaw_spd: yaw轴速度计算值
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(armor_placement.location);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = (float)0;
            send.pitch_angle = (float)s_pitch;
            send.pitch_speed = (float)0;
            send.shoot_mode = ShootMode::FOLLOW;
        }
        else {
            sort(new_detections.begin(), new_detections.end(), 
                [](const bbox_t &a, const bbox_t &b) {
                    return a.pts[0].x > b.pts[0].x;  // 从大到小排序
                }
            ); //从右到左排序，临时措施
            armor = new_detections[0];//xy:我似乎需要先选择装甲板？这里直接选择视野里的第一块
            // pnp解算，确定观测向量
            ArmorPlacementPW armor_placement = position_transform.pnp_get_ArmorPlacementPW(armor.pts, armor.tag_id);
            Matz1 observation_vector; 
            observation_vector<<    armor_placement.x(),
                                    armor_placement.y(),
                                    armor_placement.get_angle();

            // 取角度 准备跳装甲板检测
            Matx1 filter_angle_last_state_0 = filter_target.state_predict_only(t);
            double predictArmorAngle_0 = filter_angle_last_state_0(2,0); 

            // 跳装甲板判断，如果跳，则让预测器转而预测即将跳到的装甲板。
            double delta = remainder(armor_placement.get_angle()-predictArmorAngle_0,360);
            if (delta > 90. / 2){
                turn_left_armor();
            }

            if (delta < - 90./2){
                turn_right_armor();
            }
            Matx1 estimated_state = filter_target.update_with_debug_print(observation_vector,t);
            Matz1 estimated_armor = observation_function(estimated_state);//得到装甲板的x,y,yaw


            double ft = FlightTimePredict(Pos3D(estimated_armor(0,0), estimated_armor(1,0), armor_placement.z()), robot_status.robot_speed_mps);      // ft: 预测弹丸飞行时间。通过装甲板当前空间位置
            auto now_t = std::chrono::high_resolution_clock::now();
            double process_latency = duration_cast<microseconds>(now_t - t).count() / 1e6;
            double t_delay = ft + comm_latency + process_latency;


            Matx1 state_after_latency = filter_target.state_predict_only(t_delay);
            Matz1 armor_after_latency = observation_function(state_after_latency);
            Eigen::Vector2d armor_velocity = get_armor_velocity(state_after_latency);
            double v_x = armor_velocity(0);
            double v_y = armor_velocity(1);

            Pos3D s_pw{armor_after_latency(0,0),armor_after_latency(1,0),armor_placement.z()}; // s_pw: ft后预测点
            speed_pc = position_transform.pw_to_pc(s_pw);
            Eigen::Vector2d r_vec(s_pw(0), s_pw(1));                                              // 目标装甲板位矢
            Eigen::Vector2d v_vec(v_y, -v_x);                                             // 目标装甲板速度
            s_pw(2, 0) -= TrajectoryCompensation(s_pw, robot_status.robot_speed_mps);                 // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(s_pw);                                           // point camera: 目标在相机坐标系下的坐标
            aim_pc = s_pc;
            double s_yaw_spd = -(r_vec.dot(v_vec)) / (r_vec.norm() * r_vec.norm()) / M_PI * 180.;     // s_yaw_spd: yaw轴速度计算值
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(s_pw);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = (float)s_yaw_spd;
            send.pitch_angle = (float)s_pitch;
        }

        filter_target.printStateInfo();

        // 预测器可视化
            if (true)
            {
                static const cv::Scalar colors[4] = {{255, 0, 0}, {0, 0, 255}, {255, 255, 255}, {0, 255, 0}};
                cv::Mat im2show = data->frame.clone();
                cv::line(im2show, armor.pts[0], armor.pts[1], colors[2], 2);
                cv::line(im2show, armor.pts[1], armor.pts[2], colors[2], 2);
                cv::line(im2show, armor.pts[2], armor.pts[3], colors[2], 2);
                cv::line(im2show, armor.pts[3], armor.pts[0], colors[2], 2);

                // 使用行列式方法求解对角线交点
                float a0 = armor.pts[2].y - armor.pts[0].y;
                float b0 = armor.pts[0].x - armor.pts[2].x;
                float c0 = armor.pts[2].x * armor.pts[0].y - armor.pts[0].x * armor.pts[2].y;

                float a1 = armor.pts[3].y - armor.pts[1].y;
                float b1 = armor.pts[1].x - armor.pts[3].x;
                float c1 = armor.pts[3].x * armor.pts[1].y - armor.pts[1].x * armor.pts[3].y;

                float D = a0 * b1 - a1 * b0;

                cv::Point2f center;
                if (std::abs(D) < 1e-6) {
                    // 如果对角线近乎平行，返回四个点的均值
                    center = (armor.pts[0] + armor.pts[1] + armor.pts[2] + armor.pts[3]) / 4.0f;
                } else {
                    center.x = (b0 * c1 - b1 * c0) / D;
                    center.y = (c0 * a1 - c1 * a0) / D;
                }

                cv::Point2f speed_vec = position_transform.projectPoint(speed_pc);
                cv::Point2f aim_vec = position_transform.projectPoint(aim_pc);
                
                cv::line(im2show, center, speed_vec, colors[1], 2);
                cv::line(im2show, speed_vec, aim_vec, colors[3], 2);

                cv::putText(im2show, std::to_string(armor.tag_id), armor.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[armor.color_id]);
                cv::imshow("predictor_debug", im2show);
                cv::waitKey(1);
            }
    }
};