/**
 * @file MultiPolicyPredictor.cpp
 * @brief 多策略预测器实现 - 整合完整预测系统的主要实现
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 实现功能：
 * 1. 预测流程的完整执行逻辑
 * 2. 装甲板筛选和目标匹配算法
 * 3. 多种可视化显示实现
 * 4. 机器人控制指令生成
 */

#include "MultiPolicyPredictor_submodule.hpp"

namespace predict
{
    /**
     * @brief 构造函数 - 初始化多策略预测器的所有组件
     * @param comm_latency_ 通信延迟时间 (毫秒)
     * @param shoot_latency_ 发射延迟时间 (毫秒)
     * @param debug_ 调试模式标志
     * @param show_ 显示模式标志
     * @param plot_ 绘图模式标志
     * @param adjust_ 参数调整模式标志
     * @details 初始化跟踪器和规划器，设置各种显示和调试选项
     *          CoordTransformer 已在 main 中初始化，直接使用单例
     */ 
    MultiPolicyPredictorSubModule::MultiPolicyPredictorSubModule(const MultiPolicyPredictorConfig& config)
    : SubModule(SubModuleName::MULTI_POLICY_PREDICTOR),
      config_(config),
      coord_transformer(CoordTransformer::Get()),
      autoaim_mode_counter(0),
      in_autoaim_mode(false),
      fixed_target_id(0)
    {
        for (int i = 0; i < NUM_TRACKER; ++i) {
            trackers[i].set_debug(config_.debug.log_text);
            trackers[i].set_adjust(config_.adjust_tracker_noise);
        }

        if (config_.adjust_mode) {
            cv::namedWindow("predictor trackbar", cv::WINDOW_AUTOSIZE);
        
            cv::createTrackbar("in_autoaim_mode", "predictor trackbar", &in_autoaim_mode_adjust, 1, 0);
        }

        LOGM_S("[MultiPolicyPredictorSubModule] construction completed");
    }

    SubModuleResult MultiPolicyPredictorSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                           const pipeline::BasicTask* parent)
    {
        auto t1 = std::chrono::steady_clock::now();

        //LOGM_S("[MultiPolicyPredictorSubModule] ready");
        
        // 执行预测算法
        predict(data);
        
        auto t2 = std::chrono::steady_clock::now();

        // 调试信息
        if (config_.debug.log_text)
        {
            // auto &send = data->robotcommand;
            // LOGM_S("[MultiPolicyPredictorSubModule] pitch %6.2f, yaw %6.2f, dist %4.1f",
            //        send.pitch_angle, send.yaw_angle,
            //        (float)send.distance / 10);
        }
        
        // 显示结果（如果需要）
        if (config_.debug.show_image)
        {
            // 预测模块的显示逻辑（如果需要的话）
        }

        auto t3 = std::chrono::steady_clock::now();
        // LOGM_S(
        //     "LinearPredictorSubModule Predict %.2lfms Show %.2lfms", 
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()*1000,
        //     std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()*1000
        // );
        
        // 预测总是成功的，返回 true
        return SubModuleResult::SUCCESS;
    }

    /**
     * @brief 主预测函数 - 执行完整的预测流程
     * @param data 线程数据包，包含检测结果、传感器数据、图像等
     * @details 完整的预测流程：
     *          1. 更新坐标变换矩阵
     *          2. 根据跟踪状态执行不同的处理逻辑
     *          3. 装甲板筛选和目标匹配
     *          4. 执行跟踪和预测计算
     *          5. 生成控制指令
     *          6. 可视化显示（可选）
     */
    void MultiPolicyPredictorSubModule::predict(std::shared_ptr<ThreadDataPack> data)
    {
        // LOGT_S();

        // === 提取数据包信息 ===
        auto &detected_armors = data->bboxes;          // 检测到的装甲板列表
        auto attitude_yaw = data->attitude.yaw() / 180 * M_PI;      // 机器人偏航角（转换为弧度）
        auto attitude_pitch = data->attitude.pitch() / 180 * M_PI;  // 机器人俯仰角（转换为弧度）
        auto tp = data->time;                          // 当前时间戳
        auto &send = data->robotcommand;               // 机器人控制指令结构体
        auto robot_status = data->robotstatus;         // 机器人状态信息
        auto R_world2imu = data->attitude.R_world2imu(); // 世界坐标系到IMU坐标系的旋转矩阵
        auto &has_fixed_target = data->has_fixed_target;   // 是否包含固定的目标

        // === 自瞄模式切换判断逻辑 ===
        if (in_autoaim_mode) {
            if (robot_status.program_mode != ProgramMode::AUTO_AIM) {
                autoaim_mode_counter--;
            }
            else {
                autoaim_mode_counter = max_autoaim_mode_counter;
            }
            
            if (autoaim_mode_counter <= 0) {
                in_autoaim_mode = false;
                autoaim_mode_counter = 0;
            }
        }
        else {
            if (robot_status.program_mode == ProgramMode::AUTO_AIM) {
                autoaim_mode_counter++;
            }
            else {
                autoaim_mode_counter = 0;
            }
            
            if (autoaim_mode_counter >= max_autoaim_mode_counter) {
                in_autoaim_mode = true;
                autoaim_mode_counter = max_autoaim_mode_counter;
            }
        }

        if (config_.adjust_mode) {
            in_autoaim_mode = in_autoaim_mode_adjust > 0;
        }

        if (config_.debug.log_text) {
            std::cout << "in_autoaim_mode: " << in_autoaim_mode 
                 << std::endl;
        }

        // === 装甲板筛选和优先级排序 ===
        bool need_new_armors = false;
        for (int i = 0; i < NUM_TRACKER; ++i) {
            if (trackers[i].get_tracker_state() == TrackingState::IDLE) {
                need_new_armors = true;
                break;
            }
        }

        std::vector<std::pair<bbox_t, double>> new_armors;
        std::vector<bbox_t> armors_in_tracking;

        // todo: 多个跟踪器实例test
        for (const auto &armor : detected_armors) {
            // 根据颜色和距离筛选装甲板�?并且如果装甲板已经在跟踪中，则加入到 armors_in_tracking 列表，否则筛选角点来源为传统视觉的装甲板加入到 new_armors 列表
            if (armor.color_id == (robot_status.enemy_color==EnemyColor::BLUE)) {
                float yaw_in_camera;
                Eigen::Matrix<double, 4, 1> measurement;
                bool success = coord_transformer.pnp_get_measurement(armor.pts, armor.tag_id, armor.color_id,
                                                                                    attitude_yaw, R_world2imu, yaw_in_camera, measurement);
                Pos3D m_pw(measurement(1, 0), measurement(0, 0), measurement(2, 0));
                double dist = distance_3D(m_pw);
                double height = m_pw(2, 0);

                if (dist < max_distance_accept && abs(measurement(2, 0)) < max_yaw_accept && abs(height) < max_height_accept) {
                    bool already_in_tracking = false;
                    for (int i = 0; i < NUM_TRACKER; ++i) {
                        if (trackers[i].get_tracker_state() != TrackingState::IDLE && armor.tag_id == tracked_armors[i].tag_id) {
                            already_in_tracking = true;
                            break;
                        }
                    }

                    if (already_in_tracking) {
                        armors_in_tracking.push_back(armor);
                    }
                    else {
                        if (need_new_armors) {
                            if (armor.source == DetectionSource::TRADITIONAL) {
                                new_armors.push_back(std::make_pair(armor, dist));
                            }
                        }
                    }
                }
            }
        }

        if (need_new_armors) {
            // 根据距离和像素中心位置对 new_armors 进行排序，优先考虑距离较近且位于图像中心的装甲板
            std::sort(new_armors.begin(), new_armors.end(), 
                [this](std::pair<bbox_t, double>& a, std::pair<bbox_t, double>& b) {
                    cv::Point2f centerA = points_center(a.first.pts);
                    float sdA = std::sqrt(std::pow(centerA.x - cx, 2) + std::pow(centerA.y - cy, 2));
                    float scoreA = (1 - dist_weight) * (sdA / maxSD) + dist_weight * (a.second / max_distance_accept);

                    cv::Point2f centerB = points_center(b.first.pts);
                    float sdB = std::sqrt(std::pow(centerB.x - cx, 2) + std::pow(centerB.y - cy, 2));
                    float scoreB = (1 - dist_weight) * (sdB / maxSD) + dist_weight * (b.second / max_distance_accept);

                    return scoreA < scoreB;
                });
        }

        for (int i = 0; i < NUM_TRACKER; ++i) {
            auto &tracker = trackers[i];
            auto &tracked_measurement = tracked_measurements[i];
            auto &secondary_tracked_measurement = secondary_tracked_measurements[i];
            auto &tracked_armor = tracked_armors[i];

            // === 根据跟踪器状态执行不同逻辑 ===
            if (tracker.get_tracker_state() == TrackingState::IDLE) {                
                // === 空闲状态：寻找新目标或重置系统 ===
                if (new_armors.empty()) {
                    if (config_.debug.log_text)
                        std::cout << "[predict] empty detection" << std::endl;
                }
                else {
                    // 检测到装甲板，开始新的跟踪
                    tracked_armor = new_armors.front().first; // 选择排序后的第一个装甲板作为跟踪目标

                    // 从 new_armors 中移除已选中的装甲板id，避免多个跟踪器选择同一装甲板id
                    new_armors.erase(
                        std::remove_if(new_armors.begin(), new_armors.end(), [&tracked_armor](const std::pair<bbox_t, double>& b) {
                            return b.first.tag_id == tracked_armor.tag_id; 
                        }),
                        new_armors.end()
                    );

                    // 通过PnP算法获取装甲板的3D位置和姿态
                    float yaw_in_camera;
                    bool success = coord_transformer.pnp_get_measurement(tracked_armor.pts, tracked_armor.tag_id, tracked_armor.color_id,
                                                                                attitude_yaw, R_world2imu, yaw_in_camera, tracked_measurement);

                    if (!success) {
                        if (config_.debug.log_text)
                            std::cout << "[predict] pnp failed for initial target" << std::endl;

                        return;
                    }
                    
                    // 重置跟踪器并初始化目标
                    auto &target = tracker.reset_target(tracked_measurement, tp);
                    
                    if (config_.debug.log_text)
                        std::cout << "[predict] start tracking" << std::endl;
                }
            }
            else {
                // === 非空闲状态：执行跟踪和预测 ===
                // 注意：以下逻辑针对单个车辆进行处理

                // === 装甲板筛选和匹配 ===
                // 寻找与当前跟踪目标相同ID的装甲板，优先考虑上次跟踪的装甲板并且来源于传统视觉
                int same_id_armor_count = 0;         // 同ID装甲板数量
                double min_position_diff = DBL_MAX;  // 最小位置差
                bbox_t selected_armor;               // 选中的装甲板
                Eigen::Matrix<double, 4, 1> selected_measurement; // 选中装甲板的测量值
                bbox_t secondary_armor;               // 备选装甲板
                Eigen::Matrix<double, 4, 1> secondary_measurement; // 备选装甲板的测量值

                // 遍历所有检测到的装甲板
                for (const auto &armor : armors_in_tracking) {
                    if (armor.tag_id == tracked_armor.tag_id && armor.color_id == tracked_armor.color_id) {
                        // 找到同ID装甲板，进行PnP解算
                        float yaw_in_camera;
                        Eigen::Matrix<double, 4, 1> measured_measurement;
                        bool success = coord_transformer.pnp_get_measurement(armor.pts, armor.tag_id, tracked_armor.color_id, 
                                                                                attitude_yaw, R_world2imu, yaw_in_camera, measured_measurement);

                        if (!success) {
                            continue;
                        }
                        
                        // 计算位置变化
                        Eigen::Matrix<double, 3, 1> measured_pw(measured_measurement(1, 0), measured_measurement(0, 0), measured_measurement(2, 0));
                        Eigen::Matrix<double, 3, 1> tracked_pw(tracked_measurement(1, 0), tracked_measurement(0, 0), tracked_measurement(2, 0));

                        same_id_armor_count++;

                        // 选中的装甲板为位置变化最小的，备选装甲板是次小的
                        double pw_diff = (tracked_pw - measured_pw).norm();
                        if (same_id_armor_count == 1) {
                            if (pw_diff < min_position_diff) {
                                min_position_diff = pw_diff;
                                selected_armor = armor;
                                selected_measurement = measured_measurement;
                            }
                        }
                        else {
                            if (pw_diff < min_position_diff) {
                                secondary_armor = selected_armor;
                                secondary_measurement = selected_measurement;

                                min_position_diff = pw_diff;
                                selected_armor = armor;
                                selected_measurement = measured_measurement;
                            }
                            else {
                                secondary_armor = armor;
                                secondary_measurement = measured_measurement;
                            }
                        }

                        if (same_id_armor_count == 2) {
                            break;
                        }
                    }
                }

                if (config_.debug.log_text)
                    std::cout << "[predict] same id armor count: " << same_id_armor_count << std::endl;

                // 更新跟踪目标，优先选择来源于传统视觉的装甲板
                if (same_id_armor_count == 1) {
                    tracked_armor = selected_armor;
                    tracked_measurement = selected_measurement;
                    secondary_tracked_measurement = secondary_measurement;
                }
                else if (same_id_armor_count == 2) {
                    if (selected_armor.source == DetectionSource::TRADITIONAL) {
                        tracked_armor = selected_armor;
                        tracked_measurement = selected_measurement;
                        secondary_tracked_measurement = secondary_measurement;
                    }
                    else {
                        if (secondary_armor.source == DetectionSource::TRADITIONAL) {
                            tracked_armor = secondary_armor;
                            tracked_measurement = secondary_measurement;
                            secondary_tracked_measurement = selected_measurement;
                        }
                        else {
                            tracked_armor = selected_armor;
                            tracked_measurement = selected_measurement;
                            secondary_tracked_measurement = secondary_measurement;
                        }
                    }
                }

                // === 执行跟踪更新 ===
                auto &target = tracker.track(tracked_measurement, secondary_tracked_measurement, same_id_armor_count, tracked_armor.tag_id, tp, attitude_yaw);
                // auto &target = tracker.track(tracked_measurement, secondary_tracked_measurement, same_id_armor_count, 8, tp, attitude_yaw);

            }
        }

        // 选择目标车辆进行云台发射规划
        if (in_autoaim_mode) {
            if (!has_fixed_target) {
                int selected_target_id = reset_target_for_plan(data->target, attitude_yaw, R_world2imu);

                if (selected_target_id == -1) {
                    fixed_target_id = 0;
                    has_fixed_target = false;
                }
                else {
                    fixed_target_id = selected_target_id;
                    has_fixed_target = true;
                }
            }
            else {
                auto &tracker = trackers[fixed_target_id];

                if (trackers[fixed_target_id].get_tracker_state() != TrackingState::IDLE) {
                    auto &tracked_armor = tracked_armors[fixed_target_id];
                    const Target &target = tracker.get_target();

                    data->target = target;
                    data->target.tracked_armor = tracked_armor;

                    int id = tracker.match_armor_id(target.tracked_measurement);
                    auto angle = mathutils::limit_rad(target.tracked_state(6, 0) + id * M_PI_2);
                    Eigen::Matrix<double, 3, 1> estimated_armor_pos = tracker.h_armor_xyz(target.tracked_state, id);

                    data->target.estimated_armor_m << estimated_armor_pos(0, 0), estimated_armor_pos(1, 0), estimated_armor_pos(2, 0), angle;

                    has_fixed_target = true;
                }
                else {
                    int selected_target_id = reset_target_for_plan(data->target, attitude_yaw, R_world2imu);

                    if (selected_target_id == -1) {
                        fixed_target_id = 0;
                        has_fixed_target = false;
                    }
                    else {
                        fixed_target_id = selected_target_id;
                        has_fixed_target = true;
                    }
                }
            }
        }
        else {
            has_fixed_target = false;

            int selected_target_id = reset_target_for_plan(data->target, attitude_yaw, R_world2imu);

            if (selected_target_id == -1) {
                fixed_target_id = 0;
            }
            else {
                fixed_target_id = selected_target_id;
            }
        }
        
    }

    int MultiPolicyPredictorSubModule::select_target_id(double attitude_yaw, const Eigen::Matrix3d &R_world2imu) {
        std::vector<std::tuple<bbox_t, double, int>> candidate_armors;
        int selected_target_id = 0;

        for (int i = 0; i < NUM_TRACKER; ++i) {
            if (trackers[i].get_tracker_state() == TrackingState::IDLE) {
                continue;
            }

            auto &armor = tracked_armors[i];
    
            float yaw_in_camera;
            Eigen::Matrix<double, 4, 1> measurement;
            bool success = coord_transformer.pnp_get_measurement(armor.pts, armor.tag_id, armor.color_id,
                                                                                attitude_yaw, R_world2imu, yaw_in_camera, measurement);
            Pos3D m_pw(measurement(1, 0), measurement(0, 0), measurement(2, 0));
            double dist = distance_3D(m_pw);

            candidate_armors.push_back(std::make_tuple(armor, dist, i));
        }

        if (candidate_armors.empty()) {
            selected_target_id = -1; // 没有可选目标
        }
        else {
            // 根据距离和像素中心位置对 candidate_armors 进行排序，优先考虑距离较近且位于图像中心的装甲板
            std::sort(candidate_armors.begin(), candidate_armors.end(), 
                [this](std::tuple<bbox_t, double, int>& a, std::tuple<bbox_t, double, int>& b) {
                    cv::Point2f centerA = points_center(std::get<0>(a).pts);
                    float sdA = std::sqrt(std::pow(centerA.x - cx, 2) + std::pow(centerA.y - cy, 2));
                    float scoreA = (1 - dist_weight) * (sdA / maxSD) + dist_weight * (std::get<1>(a) / max_distance_accept);

                    cv::Point2f centerB = points_center(std::get<0>(b).pts);
                    float sdB = std::sqrt(std::pow(centerB.x - cx, 2) + std::pow(centerB.y - cy, 2));
                    float scoreB = (1 - dist_weight) * (sdB / maxSD) + dist_weight * (std::get<1>(b) / max_distance_accept);

                    return scoreA < scoreB;
                });

            selected_target_id = std::get<2>(candidate_armors.front());
        }

        return selected_target_id;
    }


    int MultiPolicyPredictorSubModule::reset_target_for_plan(Target &data_target, double attitude_yaw, const Eigen::Matrix3d &R_world2imu) {
        int selected_target_id = select_target_id(attitude_yaw, R_world2imu);

        if (selected_target_id == -1) {
            data_target = trackers[0].get_target();
        }
        else {
            auto &tracker = trackers[selected_target_id];
            auto &tracked_armor = tracked_armors[selected_target_id];
            const Target &target = tracker.get_target();

            data_target = target;
            data_target.tracked_armor = tracked_armor;

            int id = tracker.match_armor_id(data_target.tracked_measurement);
            auto angle = mathutils::limit_rad(data_target.tracked_state(6, 0) + id * M_PI_2);
            Eigen::Matrix<double, 3, 1> estimated_armor_pos = tracker.h_armor_xyz(data_target.tracked_state, id);

            data_target.estimated_armor_m << estimated_armor_pos(0, 0), estimated_armor_pos(1, 0), estimated_armor_pos(2, 0), angle;
        }

        return selected_target_id;
    }

}
