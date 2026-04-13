//
// LinearPredictorSubModule - Merged PredictSubModule and LinearPredictor
// Combines pipeline integration and prediction algorithm in one class
//

#include "planner_submodule.hpp"

namespace plan
{
    PlannerSubModule::PlannerSubModule(const PlannerConfig& config,
        pipeline::bridge::PlannerToSerialBridge &message_bridge, const std::string planner_param)
        : SubModule(SubModuleName::PLANNER), config_(config), planner_bridge(message_bridge),
          planner(planner_param, config.debug.log_text),
          coord_transformer(CoordTransformer::Get())
    {
        LOGM_S("[Planner] construction completed");
    }

    PlannerSubModule::command_array_t PlannerSubModule::generate_command_array(const RobotCommand& command) {
        constexpr std::chrono::microseconds plan_period{2000};
        command_array_t commands;
        for (size_t i = 0; i < CMDARRAYLENGTH; ++i) {
            commands[i] = RobotCommand{
                command.distance,
                command.yaw_angle+i*command.yaw_speed*float(plan_period.count())/1e6f,
                command.yaw_speed,
                command.yaw_acc,
                command.pitch_angle+i*command.pitch_speed*float(plan_period.count())/1e6f,
                command.pitch_speed,
                command.pitch_acc,
                command.fire_enable,
                command.target_id
            };
        }
        return commands;
    }

    SubModuleResult PlannerSubModule::process(std::shared_ptr<ThreadDataPack> data, 
                                     const pipeline::BasicTask* parent)
    {
        auto attitude_yaw = data->attitude.yaw() / 180 * M_PI;      // 机器人偏航角（转换为弧度）
        auto attitude_pitch = data->attitude.pitch() / 180 * M_PI;  // 机器人俯仰角（转换为弧度）
        auto tp = data->time;                          // 当前时间戳
        auto &send = data->robotcommand;               // 机器人控制指令结构体
        auto robot_status = data->robotstatus;         // 机器人状态信息
        auto R_world2imu = data->attitude.R_world2imu(); // 世界坐标系到IMU坐标系的旋转矩阵
        auto &target = data->target;
        auto has_fixed_target = data->has_fixed_target; 

        if (target.predictor_state == TrackingState::IDLE) {
            planner.planner_reset();
        }
        else if (target.predictor_state == TrackingState::DETECTING) {
            planner.aim_target_init();
        }

        if (target.predictor_state != TrackingState::IDLE) {
            auto &plan = planner.make_plan(target, robot_status.robot_speed_mps,
                attitude_yaw, attitude_pitch, R_world2imu, tp);

            // 输出数据用于绘图分析（可选）
            if (config_.plot)
                output_data_to_plot(target, plan, data);
        }

        auto &plan = planner.get_plan();

        // === 更新发送给下位机的控制指令 ===
        update_information_to_send(has_fixed_target, target, plan, send, attitude_yaw, attitude_pitch);

        // === 可视化显示（可选） ===
        if (config_.debug.show_image) {
            show_real_world(target, plan, data, R_world2imu);  // 显示真实世界视图
            show_sim(target, plan);                           // 显示仿真俯视图
        }

        pipeline::bridge::PlannerToSerialMessage msg{
            generate_command_array(data->robotcommand),
            data->attitude,
            plan_period
        };
        planner_bridge.send(msg);
        return SubModuleResult::SUCCESS;
    }

    /**
     * @brief 更新发送给下位机的控制指令
     * @param plan 预测计划结构体
     * @param send 机器人控制指令结构体（输出）
     * @details 将预测结果转换为机器人可执行的控制指令
     */
    void PlannerSubModule::update_information_to_send(const bool has_fixed_target, const Target &target, const Plan &plan, RobotCommand &send,
                                                        float attitude_yaw, float attitude_pitch)
    {
        if (!has_fixed_target) {
            send.distance = 0.0f;
            send.fire_enable = 0;
            send.pitch_angle = 0.0f;
            send.pitch_speed = 0.0f;
            send.pitch_acc = 0.0f;
            send.yaw_angle = 0.0f;
            send.yaw_speed = 0.0f;
            send.yaw_acc = 0.0f;

            if (plan.aimed_target_type != AimedTargetType::NONE) {
                send.target_id = target.tracked_armor.tag_id;
            }
            else {
                send.target_id = 0;
            }

            if (config_.debug.log_text)
                std::cout << "[predictor] plan: not fixed" << std::endl;
        }
        else {
            if (plan.aimed_target_type != AimedTargetType::NONE) {
                // 有有效目标时，更新控制指令
                send.distance = plan.target_distance;                          // 目标距离
                send.fire_enable = plan.fire_enable;                          // 射击使能

                // if (send.fire_enable == 3) {
                //     LOGT_S();
                //     std::cout << "fire 11111111111111111111111111111111" << std::endl;
                // }

                send.pitch_angle = (plan.target_pitch - attitude_pitch) / M_PI * 180.0f;         // 俯仰角（转换为度数）
                send.pitch_speed = plan.target_pitch_speed;                   // 俯仰角速度
                send.pitch_acc = plan.target_pitch_acc;
                send.yaw_angle = (plan.target_yaw - attitude_yaw) / M_PI * 180.0f;            // 偏航角（转换为度数）
                send.yaw_speed = plan.target_yaw_speed;                       // 偏航角速度
                send.yaw_acc = plan.target_yaw_acc;

                send.target_id = target.tracked_armor.tag_id;

                if (config_.debug.log_text)
                    std::cout << "[predictor] plan: sent" << std::endl;
            }
            else {
                // 无有效目标时，清除所有控制指令与目标ID
                send.distance = 0.0f;
                send.fire_enable = 0;
                send.pitch_angle = 0.0f;
                send.pitch_speed = 0.0f;
                send.pitch_acc = 0.0f;
                send.yaw_angle = 0.0f;
                send.yaw_speed = 0.0f;
                send.yaw_acc = 0.0f;
                send.target_id = 0;

                if (config_.debug.log_text)
                    std::cout << "[predictor] plan: none" << std::endl;
            }
        }

        // send.fire_enable = 1;
        // send.pitch_angle = 0;
        // send.yaw_angle = 0;
        // send.target_id = 2;
        // send.distance = 1;
    }

    /**
    * @brief 输出数据用于绘图分析
    * @param target 目标跟踪状态
    * @param plan 预测计划
    * @details 输出关键跟踪和预测数据，用于离线分析和系统调优
    *          当前实现中的输出语句已被注释，可根据需要启用特定数据的输出
    */
    void PlannerSubModule::output_data_to_plot(const Target &target, const Plan &plan, std::shared_ptr<ThreadDataPack> data) 
    {
        LOGT_S();

        // std::cout << data->robotstatus.robot_speed_mps << std::endl;

        // std::cout << data->attitude.yaw() << std::endl;
        // std::cout << data->attitude.pitch() << std::endl;

        // std::cout << (tracked_armor.source == DetectionSource::TRADITIONAL ? 1 : 0) << std::endl;

        // std::cout << target.tracked_measurement(0, 0) << std::endl;
        // std::cout << target.tracked_measurement(1, 0) << std::endl;
        // std::cout << target.tracked_measurement(2, 0) << std::endl;
        // std::cout << target.tracked_measurement(3, 0) << std::endl;

        // std::cout << target.tracked_measurement(0, 0) + target.tracked_state(8, 0) * cos(target.tracked_state(6, 0)) << std::endl;
        // std::cout << target.tracked_measurement(1, 0) + target.tracked_state(8, 0) * sin(target.tracked_state(6, 0)) << std::endl;
        // std::cout << target.tracked_state(0, 0) - target.tracked_state(8, 0) * cos(target.tracked_state(6, 0)) << std::endl;
        // std::cout << target.tracked_state(2, 0) - target.tracked_state(8, 0) * sin(target.tracked_state(6, 0)) << std::endl;
        // std::cout << target.tracked_measurement(2, 0) << std::endl;
        // std::cout << target.tracked_measurement(3, 0) << std::endl;

        // std::cout << static_cast<int>(target.predictor_state) << std::endl;
        // std::cout << target.ab_counter << std::endl;

        // std::cout << target.yaw_state(0, 0) << std::endl;
        // std::cout << target.yaw_state(1, 0) << std::endl;

        // std::cout << target.armor_y_state(0, 0) << std::endl;
        // std::cout << target.armor_y_state(1, 0) << std::endl;

        // std::cout << target.armor_x_state(0, 0) << std::endl;
        // std::cout << target.armor_x_state(1, 0) << std::endl;

        // std::cout << target.armor_z_state(0, 0) << std::endl;
        // std::cout << target.armor_z_state(1, 0) << std::endl;

        // std::cout << target.tracked_state(0, 0) << std::endl;
        // std::cout << target.tracked_state(1, 0) << std::endl;
        // std::cout << target.tracked_state(2, 0) << std::endl;
        // std::cout << target.tracked_state(3, 0) << std::endl;
        // std::cout << target.tracked_state(4, 0) << std::endl;
        // std::cout << target.tracked_state(5, 0) << std::endl;
        // std::cout << target.tracked_state(6, 0) << std::endl;
        // std::cout << target.tracked_state(7, 0) << std::endl;
        // std::cout << target.tracked_state(8, 0) << std::endl;
        // std::cout << target.tracked_state(9, 0) << std::endl;
        // std::cout << target.tracked_state(10, 0) << std::endl;

        // std::cout << target.vehicle_model_trust << std::endl;

        // std::cout << plan.aimed_armor_pos(0, 0) << std::endl;
        // std::cout << plan.aimed_armor_pos(1, 0) << std::endl;
        // std::cout << plan.aimed_armor_pos(2, 0) << std::endl;

        // std::cout << plan.target_yaw << std::endl;
        // std::cout << plan.target_yaw / M_PI * 180.0f << std::endl;
        // std::cout << plan.target_yaw_speed << std::endl;
        // std::cout << plan.target_yaw_acc << std::endl;

        // std::cout << plan.target_pitch / M_PI * 180.0f << std::endl;
        // std::cout << plan.target_pitch_speed / M_PI * 180.0f << std::endl;
        // std::cout << plan.target_pitch_acc / M_PI * 180.0f << std::endl;

        std::cout << plan.fire_enable << std::endl;
    }

    // === 枚举转字符串辅助函数 ===

    /**
    * @brief 跟踪状态枚举转字符串
    * @param x 跟踪状态枚举值
    * @return 对应的字符串描述
    */
    std::string PlannerSubModule::TrackingState2String(const TrackingState & x) 
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
    std::string PlannerSubModule::AimedTargetType2String(const AimedTargetType & x) 
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
    std::string PlannerSubModule::UpdatingModelType2String(const UpdatingModelType & x) 
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
    void PlannerSubModule::show_real_world(const Target &target, const Plan &plan, 
                                std::shared_ptr<ThreadDataPack> &data,const Eigen::Matrix3d &R_world2imu)
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
        Eigen::Matrix<double, 4, 1> estimated_armor_m = target.estimated_armor_m;
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
        if (target.predictor_state == TrackingState::TRACKING || target.predictor_state == TrackingState::DETECTING) {
            auto &tracked_armor = target.tracked_armor;

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
        cv::putText(im2show, "dh: " + std::to_string(target.tracked_state(10, 0)), zero + 15 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);
        cv::putText(im2show, "dl: " + std::to_string(target.tracked_state(9, 0)), zero + 14 * offset, cv::FONT_HERSHEY_SIMPLEX, 1, colors[1]);

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
    void PlannerSubModule::show_sim(const Target &target, const Plan &plan)
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
