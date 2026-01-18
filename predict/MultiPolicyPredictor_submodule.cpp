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
    MultiPolicyPredictorSubModule::MultiPolicyPredictorSubModule(int comm_latency_, int shoot_latency_,
                                                                    double pitch_comp, double yaw_comp, bool disable_vehicle_center_shoot_mode,
                                                                    bool debug_, bool show_, bool plot_, bool adjust_)
    : SubModule(SubModuleName::MULTI_POLICY_PREDICTOR),
      debug(debug_),
      show(show_),
      plot(plot_),
      adjust(adjust_),
      tracker(debug_, adjust_),
      planner(comm_latency_ / 1e3, shoot_latency_ / 1e3, pitch_comp, yaw_comp, disable_vehicle_center_shoot_mode, debug_),
      coord_transformer(CoordTransformer::Get())
    {
        
        LOGM_S("[MultiPolicyPredictorSubModule] constructing with latency: %d", comm_latency_);

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


        bool show_armor = false; // 控制是否在可视化中显示装甲板边界框

        // TODO: 多个跟踪器实例
        // === 根据跟踪器状态执行不同逻辑 ===
        if (tracker.get_tracker_state() == TrackingState::IDLE) {
            // === 空闲状态：寻找新目标或重置系统 ===
            if (detected_armors.empty()) {
                // 没有检测到装甲板，重置规划器
                planner.planner_reset();

                if (debug)
                    std::cout << "[predict] empty detection" << std::endl;
            }
            else {
                // 检测到装甲板，开始新的跟踪
                vector<bbox_t> candidate_armors_from_trad;
                vector<bbox_t> candidate_armors_from_nnet;
                for (const auto &armor : detected_armors) {
                    // TODO: temporaliy do not use enemy_color
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
                    // TODO: 添加车辆选择逻辑
                    if (candidate_armors_from_trad.empty()) {
                        tracked_armor = candidate_armors_from_nnet.front();
                    }
                    else {
                        tracked_armor = candidate_armors_from_trad.front();
                    }

                    find_target = true;
                }

                if (!find_target) {
                    // 没有检测到装甲板，重置规划器
                    planner.planner_reset();

                    if (debug)
                        std::cout << "[predict] no valid target found" << std::endl;
                }
                else {
                    show_armor = true;

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
                    
                    // 初始化规划器
                    planner.aim_target_init();
                    
                    // 生成初始预测计划
                    auto &plan = planner.make_plan(target, robot_status.robot_speed_mps,
                                attitude_yaw, attitude_pitch, R_world2imu, tp);

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
                    show_armor = true;

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
            else {
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
            auto &target = tracker.track(tracked_measurement, secondary_tracked_measurement, same_id_armor_count, tp, attitude_yaw);

            // === 生成预测计划 ===
            auto &plan = planner.make_plan(target, robot_status.robot_speed_mps,
                                        attitude_yaw, attitude_pitch, R_world2imu, tp);

            // 输出数据用于绘图分析（可选）
            if (plot)
                output_data_to_plot(target, plan);
        }

        // === 获取最终的目标和计划信息 ===
        auto &target = tracker.get_target();
        auto &plan = planner.get_plan();

        // === 更新发送给下位机的控制指令 ===
        update_information_to_send(plan, send, attitude_yaw, attitude_pitch);

        // === 可视化显示（可选） ===
        if (show) {
            show_real_world(target, plan, data, R_world2imu, show_armor);  // 显示真实世界视图
            show_sim(target, plan);                           // 显示仿真俯视图
        }
    }

    /**
     * @brief 更新发送给下位机的控制指令
     * @param plan 预测计划结构体
     * @param send 机器人控制指令结构体（输出）
     * @details 将预测结果转换为机器人可执行的控制指令
     */
    void MultiPolicyPredictorSubModule::update_information_to_send(const Plan &plan, RobotCommand &send,
                                                                   float attitude_yaw, float attitude_pitch)
    {
        if (plan.aimed_target_type != AimedTargetType::NONE) {
            // 有有效目标时，更新控制指令
            send.distance = plan.target_distance;                          // 目标距离
            send.fire_enable = plan.fire_enable;                          // 射击使能
            send.pitch_angle = (plan.target_pitch - attitude_pitch) / M_PI * 180.0f;         // 俯仰角（转换为度数）
            send.pitch_speed = plan.target_pitch_speed;                   // 俯仰角速度
            send.yaw_angle = (plan.target_yaw - attitude_yaw) / M_PI * 180.0f;            // 偏航角（转换为度数）
            send.yaw_speed = plan.target_yaw_speed;                       // 偏航角速度
            send.target_id = tracked_armor.tag_id;                        // 目标ID
        }
        else {
            // 无有效目标时，清除目标ID
            send.target_id = 0;

            if (debug)
                cout << "[predictor] target: none" << endl;
        }
    }

    /**
     * @brief 输出数据用于绘图分析
     * @param target 目标跟踪状态
     * @param plan 预测计划
     * @details 输出关键跟踪和预测数据，用于离线分析和系统调优
     *          当前实现中的输出语句已被注释，可根据需要启用特定数据的输出
     */
    void MultiPolicyPredictorSubModule::output_data_to_plot(const Target &target, const Plan &plan) 
    {
        LOGT_S();

        // cout << (tracked_armor.source == DetectionSource::TRADITIONAL ? 1 : 0) << endl;

        // cout << target.tracked_measurement(0, 0) << std::endl;
        // cout << target.tracked_measurement(1, 0) << std::endl;
        // cout << target.tracked_measurement(2, 0) << std::endl;
        // cout << target.tracked_measurement(3, 0) << std::endl;

        // cout << target.tracked_measurement(0, 0) + target.tracked_state(8, 0) * cos(target.tracked_state(6, 0)) << std::endl;
        // cout << target.tracked_measurement(1, 0) + target.tracked_state(8, 0) * sin(target.tracked_state(6, 0)) << std::endl;
        // cout << target.tracked_state(0, 0) - target.tracked_state(8, 0) * cos(target.tracked_state(6, 0)) << std::endl;
        // cout << target.tracked_state(2, 0) - target.tracked_state(8, 0) * sin(target.tracked_state(6, 0)) << std::endl;
        // cout << target.tracked_measurement(2, 0) << std::endl;
        // cout << target.tracked_measurement(3, 0) << std::endl;
        // std::cout << static_cast<int>(target.predictor_state) << std::endl;
        // cout << target.ab_counter << std::endl;

        // cout << target.yaw_state(0, 0) << endl;
        // cout << target.yaw_state(1, 0) << endl;

        // cout << target.armor_y_state(0, 0) << endl;
        // cout << target.armor_y_state(1, 0) << endl;

        // cout << target.armor_x_state(0, 0) << endl;
        // cout << target.armor_x_state(1, 0) << endl;

        // cout << target.armor_z_state(0, 0) << endl;
        // cout << target.armor_z_state(1, 0) << endl;
        
        // cout << target.tracked_state(0, 0) << std::endl;
        // cout << target.tracked_state(1, 0) << std::endl;
        // cout << target.tracked_state(2, 0) << std::endl;
        // cout << target.tracked_state(3, 0) << std::endl;
        // cout << target.tracked_state(4, 0) << std::endl;
        // cout << target.tracked_state(5, 0) << std::endl;
        // cout << target.tracked_state(6, 0) << std::endl;
        // cout << target.tracked_state(7, 0) << std::endl;
        // cout << target.tracked_state(8, 0) << std::endl;
        // cout << target.tracked_state(9, 0) << std::endl;
        // cout << target.tracked_state(10, 0) << std::endl;

        // cout << target.vehicle_model_trust << std::endl;

        // cout << plan.aimed_armor_pos(0, 0) << endl;
        // cout << plan.aimed_armor_pos(1, 0) << endl;
        // cout << plan.aimed_armor_pos(2, 0) << endl;

        // cout << plan.target_yaw << std::endl;
        // cout << plan.target_yaw_speed << std::endl;

        // cout << plan.target_pitch << std::endl;
        // cout << plan.target_pitch_speed << std::endl;

        // cout << plan.fire_enable << endl;
    }

    // === 枚举转字符串辅助函数 ===
    
    /**
     * @brief 跟踪状态枚举转字符串
     * @param x 跟踪状态枚举值
     * @return 对应的字符串描述
     */
    std::string MultiPolicyPredictorSubModule::TrackingState2String(const TrackingState & x) 
    {
        switch (x) {
            case TrackingState::IDLE: return "idle";
            case TrackingState::DETECTING: return "detecting";
            case TrackingState::TRACKING: return "tracking";
            case TrackingState::TEMP_LOST: return "temp lost";
            default: return "error";
        }
    }

    /**
     * @brief 瞄准目标类型枚举转字符串
     * @param x 瞄准目标类型枚举值
     * @return 对应的字符串描述
     */
    std::string MultiPolicyPredictorSubModule::AimedTargetType2String(const AimedTargetType & x) 
    {
        switch (x) {
            case AimedTargetType::NONE: return "NONE";
            case AimedTargetType::ARMOR_WITH_NO_MODEL: return "ARMOR_WITH_NO_MODEL";
            case AimedTargetType::ARMOR_WITH_ARMOR_MODEL: return "ARMOR_WITH_ARMOR_MODEL";
            case AimedTargetType::ARMOR_WITH_VEHICLE_MODEL: return "ARMOR_WITH_VEHICLE_MODEL";
            case AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL: return "VEHICLE_CENTER_WITH_VEHICLE_MODEL";
            default: return "error";
        }
    }

    /**
     * @brief 模型更新类型枚举转字符串
     * @param x 模型更新类型枚举值
     * @return 对应的字符串描述
     */
    std::string MultiPolicyPredictorSubModule::UpdatingModelType2String(const UpdatingModelType & x) 
    {
        switch (x) {
            case UpdatingModelType::ARMOR_MODEL: return "ARMOR_MODEL";
            case UpdatingModelType::VEHICLE_MODEL: return "VEHICLE_MODEL";
            case UpdatingModelType::BOTH: return "BOTH";
            default: return "error";
        }
    }

    /**
     * @brief 显示真实世界视图
     * @param target 目标跟踪状态
     * @param plan 预测计划
     * @param data 线程数据包（包含原始图像）
     * @param show_armor 是否显示装甲板边界框
     * @details 在原始图像上叠加显示：
     *          - 白色圆圈：估计的车辆中心位置
     *          - 绿色圆圈：测量的装甲板位置
     *          - 蓝色圆圈：估计的装甲板位置（基于整车模型）
     *          - 红色圆圈：预测的瞄准点
     *          - 白色边框：装甲板检测边界框
     *          - 文字信息：跟踪状态和估计参数
     */
    void MultiPolicyPredictorSubModule::show_real_world(const Target &target, const Plan &plan, 
                                                std::shared_ptr<ThreadDataPack> &data,const Eigen::Matrix3d &R_world2imu, bool show_armor)
    {
        cv::Point2d zero(50, 50);
        cv::Point2d right_top(800, 50);
        cv::Point2d offset(0, 50);

        static const cv::Scalar colors[3] = {{0, 0, 255}, {255, 0, 0}, {255, 255, 255}};
        cv::Mat im2show = data->frame.clone();

        // estimated center
        Pos3D pw(target.tracked_state(2, 0), target.tracked_state(0, 0), target.tracked_state(4, 0));
        Pos3D pc = coord_transformer.pw_to_pc(pw, R_world2imu);
        Pos3D pu = coord_transformer.pc_to_pu(pc);
        cv::Point2d pi(pu(0, 0), pu(1, 0));
        cv::circle(im2show, pi, 5, {255, 255, 255}, 3); // white

        // measured armor
        Pos3D pw_a(target.tracked_measurement(1, 0), target.tracked_measurement(0, 0), target.tracked_measurement(2, 0));
        Pos3D pc_a = coord_transformer.pw_to_pc(pw_a, R_world2imu);
        Pos3D pu_a = coord_transformer.pc_to_pu(pc_a);
        cv::Point2d pi_a(pu_a(0, 0), pu_a(1, 0));
        cv::circle(im2show, pi_a, 5, {0, 255, 0}, 3); // green

        // // estimated armor, armor model
        // Eigen::Matrix<double, 4, 1> estimated_armor_m;
        // estimated_armor_m << target.armor_y_state(0, 0), target.armor_x_state(0, 0), target.armor_z_state(0, 0), 0;
        // Pos3D pw_ea(estimated_armor_m(1, 0), estimated_armor_m(0, 0), estimated_armor_m(2, 0));
        // Pos3D pc_ea = coord_transformer.pw_to_pc(pw_ea, R_world2imu);
        // Pos3D pu_ea = coord_transformer.pc_to_pu(pc_ea);
        // cv::Point2d pi_ea(pu_ea(0, 0), pu_ea(1, 0));
        // cv::circle(im2show, pi_ea, 5, {255, 0, 0}, 3); // blue

        // estimated armor, vehicle model
        int id = tracker.match_armor_id(target.tracked_measurement);
        cout << "matched armor id: " << id << endl;
        auto angle = mathutils::limit_rad(target.tracked_state(6, 0) + id * M_PI_2);
        Eigen::Vector3d xyz = tracker.h_armor_xyz(target.tracked_state, id);
        Eigen::Matrix<double, 4, 1> estimated_armor_m;
        estimated_armor_m << xyz[0], xyz[1], xyz[2], angle;
        Pos3D pw_ea(estimated_armor_m(1, 0), estimated_armor_m(0, 0), estimated_armor_m(2, 0));
        Pos3D pc_ea = coord_transformer.pw_to_pc(pw_ea, R_world2imu);
        Pos3D pu_ea = coord_transformer.pc_to_pu(pc_ea);
        cv::Point2d pi_ea(pu_ea(0, 0), pu_ea(1, 0));
        cv::circle(im2show, pi_ea, 5, {255, 0, 0}, 3); // blue

        // armor target
        Pos3D pw_t(plan.aimed_armor_pos(0, 0), plan.aimed_armor_pos(1, 0), plan.aimed_armor_pos(2, 0));
        Pos3D pc_t = coord_transformer.pw_to_pc(pw_t, R_world2imu);
        Pos3D pu_t = coord_transformer.pc_to_pu(pc_t);
        cv::Point2d pi_t(pu_t(0, 0), pu_t(1, 0));
        cv::circle(im2show, pi_t, 5, {0, 0, 255}, 3); // red

        // armor bbox
        if (show_armor) {
            cv::line(im2show, tracked_armor.pts[0], tracked_armor.pts[1], colors[2], 1);
            cv::line(im2show, tracked_armor.pts[1], tracked_armor.pts[2], colors[2], 1);
            cv::line(im2show, tracked_armor.pts[2], tracked_armor.pts[3], colors[2], 1);
            cv::line(im2show, tracked_armor.pts[3], tracked_armor.pts[0], colors[2], 1); // white

            cv::putText(im2show, std::to_string(tracked_armor.tag_id), tracked_armor.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[tracked_armor.color_id]);

        }

        // states
        cv::putText(im2show, std::to_string(target.tracked_state(2, 0)), zero, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_state(0, 0)), zero + offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_state(4, 0)), zero + 2 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_state(3, 0)), zero + 3 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_state(1, 0)), zero + 4 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_state(5, 0)), zero + 5 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, "yaw: " + std::to_string(target.tracked_state(6, 0)), zero + 6 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, "v_yaw: " + std::to_string(target.tracked_state(7, 0)), zero + 7 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_state(8, 0)), zero + 8 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, "another_r: " + std::to_string(target.tracked_state(8, 0) + target.tracked_state(9, 0)), zero + 13 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, "dh: " + std::to_string(target.tracked_state(10, 0)), zero + 14 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);

        cv::putText(im2show, "measurement:" + std::to_string(target.tracked_measurement(1, 0)), zero + 9 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_measurement(0, 0)), zero + 10 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_measurement(2, 0)), zero + 11 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, std::to_string(target.tracked_measurement(3, 0)), zero + 12 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);

        cv::putText(im2show, TrackingState2String(target.predictor_state), right_top + 0 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, UpdatingModelType2String(target.updating_model_type), right_top + 1 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, AimedTargetType2String(plan.aimed_target_type), right_top + 2 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);

        cv::imshow("predictor_real_world", im2show);
        cv::waitKey(1);
    }

    /**
     * @brief 显示仿真俯视图
     * @param target 目标跟踪状态
     * @param plan 预测计划
     * @details 显示俯视角度的2D仿真图，包括：
     *          - 白色圆圈：估计的车辆中心位置
     *          - 蓝色圆圈：估计的装甲板位置
     *          - 绿色圆圈：测量的装甲板位置
     *          - 红色圆圈：预测的瞄准点
     *          - 绿色直线：装甲板朝向指示
     */
    void MultiPolicyPredictorSubModule::show_sim(const Target &target, const Plan &plan)
    {
        // 仿真图像参数设置
        int h = 1000;           // 图像高度
        int w = 1000;           // 图像宽度
        int percentage = 100;   // 坐标缩放比例
        int origin_x = 500;     // 原点X坐标
        int origin_y = 1000;    // 原点Y坐标
        cv::Mat hh = cv::Mat::zeros(1000,1000,CV_8UC3); // 创建黑色背景图像

        // === 绘制估计的车辆中心（白色圆圈） ===
        cv::Point2d pw(500-target.tracked_state(2, 0)*percentage, origin_y-target.tracked_state(0, 0)*percentage);
        cv::circle(hh, pw, 5, {255, 255, 255}, 3);  // 白色圆圈

        // === 绘制估计的装甲板位置（蓝色圆圈） ===
        cv::Point2d pa(500-(target.tracked_state(2, 0) - target.tracked_state(8, 0) * sin(target.tracked_state(6, 0)))*percentage, 
                        origin_y-(target.tracked_state(0, 0) - target.tracked_state(8, 0) * cos(target.tracked_state(6, 0)))*percentage);
        cv::circle(hh, pa, 5, {255, 0, 0}, 3);  // 蓝色圆圈

        // === 绘制测量的装甲板位置（绿色圆圈） ===
        cv::Point2d pa_m(500-target.tracked_measurement(1, 0)*percentage, origin_y-target.tracked_measurement(0, 0)*percentage);
        cv::circle(hh, pa_m, 5, {0, 255, 0}, 3);  // 绿色圆圈

        // === 绘制预测的瞄准目标（红色圆圈） ===
        cv::Point2d pa_aim(500-plan.aimed_armor_pos(0, 0)*percentage, origin_y-plan.aimed_armor_pos(1, 0)*percentage);
        cv::circle(hh, pa_aim, 5, {0, 0, 255}, 3);  // red

        // cv::Point2d pw(500-target.tracked_state(2, 0)*percentage, origin_y-target.tracked_state(0, 0)*percentage);
        // cv::circle(hh, pw, 5, {255, 0, 0}, 3);  // blue

        // // std::cout << pw << std::endl;
        
        // // cv::line(im2show, 500-target.tracked_measurement(2,0), 500-target.tracked_measurement(0,0), {255, 0, 0}, 2);
        // cv::Point2d pa(500-target.tracked_measurement(1,0)*percentage, origin_y-target.tracked_measurement(0,0)*percentage);
        // cv::circle(hh, pa, 5, {0, 255, 0}, 3);  // green

        // Eigen::Matrix<double, 4, 1> tt = planner.whole_state_2_measurement(target.tracked_state);
        // cv::Point2d pa_state(500-tt(1,0)*percentage, origin_y-tt(0,0)*percentage);
        // cv::circle(hh, pa_state, 5, {255, 0, 0}, 3);
        
        // cv::Point2d p_armor_left(500-(target.tracked_measurement(1,0) + 0.066*cos(-target.tracked_measurement(3, 0)))*percentage, 
        //                          origin_y-(target.tracked_measurement(0,0) + 0.066*sin(-target.tracked_measurement(3, 0)))*percentage);
        // cv::Point2d p_armor_right(500-(target.tracked_measurement(1,0) - 0.066*cos(-target.tracked_measurement(3, 0)))*percentage, 
        //                          origin_y-(target.tracked_measurement(0,0) - 0.066*sin(-target.tracked_measurement(3, 0)))*percentage);
        // cv::line(hh, p_armor_left, p_armor_right, {0, 255, 0}, 2);

        // cv::line(hh, pw, pa, {255, 0, 0}, 2);

        cv::Point2d origin(500,500);
        int r = 200;
        cv::Point2d point(500+r*cos(-target.tracked_measurement(3, 0)),500+r*sin(-target.tracked_measurement(3, 0)));
        cv::line(hh, origin, point, {0, 255, 0}, 2);

        // 显示仿真图像 
        cv::imshow("predictor_sim", hh);
        cv::waitKey(1);
    }
}