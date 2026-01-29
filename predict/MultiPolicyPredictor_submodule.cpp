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
    MultiPolicyPredictorSubModule::MultiPolicyPredictorSubModule(bool debug_, bool adjust_)
    : SubModule(SubModuleName::MULTI_POLICY_PREDICTOR),
      debug(debug_),
      adjust(adjust_),
      tracker(debug_, adjust_),
      coord_transformer(CoordTransformer::Get())
    {
        LOGM_S("[MultiPolicyPredictorSubModule] construction completed");
    }

    bool MultiPolicyPredictorSubModule::should_skip(std::shared_ptr<ThreadDataPack> data) const
    {
        if(data->submodule_results[static_cast<uint8_t>(SubModuleName::DETECT)] != SubModuleResult::SUCCESS || 
           data->submodule_results[static_cast<uint8_t>(SubModuleName::SENSOR)] != SubModuleResult::SUCCESS
        )
            return true;
        return false;
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
        if (_debugprint)
        {
            // auto &send = data->robotcommand;
            // LOGM_S("[MultiPolicyPredictorSubModule] pitch %6.2f, yaw %6.2f, dist %4.1f",
            //        send.pitch_angle, send.yaw_angle,
            //        (float)send.distance / 10);
        }
        
        // 显示结果（如果需要）
        if (_imgshow)
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

        // todo: 多个跟踪器实例
        // === 根据跟踪器状态执行不同逻辑 ===
        if (tracker.get_tracker_state() == TrackingState::IDLE) {
            // === 空闲状态：寻找新目标或重置系统 ===
            if (detected_armors.empty()) {
                if (debug)
                    std::cout << "[predict] empty detection" << std::endl;
            }
            else {
                // 检测到装甲板，开始新的跟踪
                vector<bbox_t> candidate_armors_from_trad;
                vector<bbox_t> candidate_armors_from_nnet;
                for (const auto &armor : detected_armors) {
                    // todo: temporaliy do not use enemy_color
                    // if (armor.color_id == (robot_status.enemy_color==EnemyColor::BLUE)) {
                        if (armor.source == DetectionSource::TRADITIONAL) {
                            candidate_armors_from_trad.push_back(armor);
                        }
                        else if (armor.source == DetectionSource::NEURAL_NETWORK) {
                            candidate_armors_from_nnet.push_back(armor);
                        }
                        else {
                            candidate_armors_from_nnet.push_back(armor);
                        }
                    // }
                }

                bool find_target = false;

                if (candidate_armors_from_trad.empty() && candidate_armors_from_nnet.empty()) {
                    if (debug)
                        std::cout << "[predict] no candidate armor found" << std::endl;
                }
                else {
                    // todo: 添加车辆选择逻辑
                    if (candidate_armors_from_trad.empty()) {
                        tracked_armor = candidate_armors_from_nnet.front();
                    }
                    else {
                        tracked_armor = candidate_armors_from_trad.front();
                    }

                    find_target = true;
                }

                if (!find_target) {
                    if (debug)
                        std::cout << "[predict] no valid target found" << std::endl;
                }
                else {
                    // 通过PnP算法获取装甲板的3D位置和姿态
                    float yaw_in_camera;
                    bool success = coord_transformer.pnp_get_measurement(tracked_armor.pts, tracked_armor.tag_id, tracked_armor.color_id,
                                                                                attitude_yaw, R_world2imu, yaw_in_camera, tracked_measurement);

                    if (!success) {
                        if (debug)
                            std::cout << "[predict] pnp failed for initial target" << std::endl;

                        return;
                    }
                    
                    // 重置跟踪器并初始化目标
                    auto &target = tracker.reset_target(tracked_measurement, tp);

                    data->target = target;
                    data->target.tracked_armor = tracked_armor;

                    int id = tracker.match_armor_id(target.tracked_measurement);
                    auto angle = mathutils::limit_rad(target.tracked_state(6, 0) + id * M_PI_2);
                    Eigen::Matrix<double, 3, 1> estimated_armor_pos = tracker.h_armor_xyz(target.tracked_state, id);

                    data->target.estimated_armor_m << estimated_armor_pos(0, 0), estimated_armor_pos(1, 0), estimated_armor_pos(2, 0), angle;
                    
                    if (debug)
                        std::cout << "[predict] start tracking" << std::endl;
                }
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
            for (const auto &armor : detected_armors) {
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

            if (debug)
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
            // auto &target = tracker.track(tracked_measurement, secondary_tracked_measurement, same_id_armor_count, 0, tp, attitude_yaw);

            data->target = target;
            data->target.tracked_armor = tracked_armor;

            int id = tracker.match_armor_id(target.tracked_measurement);
            auto angle = mathutils::limit_rad(target.tracked_state(6, 0) + id * M_PI_2);
            Eigen::Matrix<double, 3, 1> estimated_armor_pos = tracker.h_armor_xyz(target.tracked_state, id);

            data->target.estimated_armor_m << estimated_armor_pos(0, 0), estimated_armor_pos(1, 0), estimated_armor_pos(2, 0), angle;

        }
        
    }

}