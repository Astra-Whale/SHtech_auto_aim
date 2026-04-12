/**
 * @file Planner.cpp
 * @brief 弹道规划器实现 - 多策略云台轨迹规划的具体实现
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 实现功能：
 * 1. 四种云台策略的具体实现
 * 2. MPC云台轨迹优化
 * 3. 弹道补偿计算
 * 4. 智能射击决策逻辑
 */

#include "Planner.hpp"

// === 评测标准开关 ===
#define ASSESSMENT_CRITERIA 0

#if ASSESSMENT_CRITERIA
int vx_constant_counter = 0;
#endif

namespace predict
{
    int a=0;

    /**
     * @brief 构造函数 - 初始化弹道规划器
     * @param comm_latency_ 通信延迟时间 (秒)
     * @param shoot_latency_ 发射延迟时间 (秒)
     * @param debug_ 调试模式标志
     * @details 初始化MPC求解器和默认参数
     */
    Planner::Planner(const std::string planner_param, bool debug_)
    : armor_jump_tp(std::chrono::high_resolution_clock::now()),
      fire_enable_tp(std::chrono::high_resolution_clock::now()),
      armor_jump(false),
      debug(debug_),
      coord_transformer(CoordTransformer::Get())
    {
        cv::FileStorage fin(planner_param, cv::FileStorage::READ);

        // 读取规划器参数
        fin["planner"]["latency"] >> comm_latency;
        comm_latency /= 1e3; 
        fin["planner"]["single_shoot_latency"] >> single_shoot_latency;
        single_shoot_latency /= 1e3; 
        fin["planner"]["continue_shoot_latency"] >> continue_shoot_latency;
        continue_shoot_latency /= 1e3; 
        fin["planner"]["same_trace_threshold"] >> same_trace_threshold;
        fin["planner"]["pitch_comp"] >> pitch_comp;
        fin["planner"]["yaw_comp"] >> yaw_comp;
        fin["planner"]["disable_vehicle_center_shoot_mode"] >> disable_vehicle_center_shoot_mode;
        fin["planner"]["disable_armor_with_vehicle_shoot_mode"] >> disable_armor_with_vehicle_shoot_mode;
        fin["planner"]["consider_air_resistence"] >> consider_air_resistence;

        shoot_offset = static_cast<int>(continue_shoot_latency / DT);

        plan.aimed_armor_pos = Eigen::Matrix<double, 3, 1>::Zero();
        plan.aimed_target_type = AimedTargetType::NONE;
        plan.fire_enable = 0;
        plan.target_yaw = 0;
        plan.target_pitch = 0;
        plan.target_yaw_speed = 0;
        plan.target_pitch_speed = 0;
        plan.target_yaw_acc = 0;
        plan.target_pitch_acc = 0;

        // 云台目标轨迹求解器
        setup_yaw_solver();
        setup_pitch_solver();
    }

    /**
     * @brief 重置规划器状态
     * @details 清空瞄准位置历史和装甲板切换状态
     */
    void Planner::planner_reset()
    {
        plan.aimed_armor_pos = Eigen::Matrix<double, 3, 1>::Zero();
        last_shooted_armor_pos = plan.aimed_armor_pos;
        armor_jump = false;
        plan.aimed_target_type = AimedTargetType::NONE;
    }

    /**
     * @brief 初始化瞄准目标
     * @details 设置初始瞄准模式为无模型预测
     */
    void Planner::aim_target_init()
    {
        plan.aimed_armor_pos = Eigen::Matrix<double, 3, 1>::Zero();
        last_shooted_armor_pos = plan.aimed_armor_pos;
        armor_jump = false;
        plan.aimed_target_type = AimedTargetType::ARMOR_WITH_NO_MODEL;
    }

    /**
     * @brief 获取当前预测计划
     * @return 计划结构体的常引用
     */
    const Plan& Planner::get_plan()
    {
        return plan;
    }

    /**
     * @brief 制定预测计划 - 根据目标状态生成完整的射击计划
     * @param target 目标跟踪状态
     * @param bullet_speed 弹丸速度 (m/s)
     * @param attitude_yaw 机器人偏航角 (弧度)
     * @param attitude_pitch 机器人俯仰角 (弧度)
     * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
     * @param tp 当前时间戳
     * @return 生成的预测计划引用
     * @details 根据目标旋转速度选择预测策略，生成包含云台角度、角速度和射击决策的完整计划
     */
    const Plan& Planner::make_plan(const Target &target, const float bullet_speed,
        const double attitude_yaw, const double attitude_pitch, const Eigen::Matrix3d &R_world2imu, const TP &tp)
    {
        double rotation_speed = abs(target.yaw_state(1, 0));
        
        // 根据目标状态和旋转速度更新瞄准策略
        update_aimed_target_type(target, rotation_speed);

        // 以下代码可用于手动强制设置瞄准类型进行调试
        // plan.aimed_target_type = AimedTargetType::ARMOR_WITH_NO_MODEL;
        // plan.aimed_target_type = AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
        // plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
        // plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
        
        // === 策略1: 无模型 ===
        // 直接瞄准当前检测到的装甲板位置，适用于初始检测阶段
        if (plan.aimed_target_type == AimedTargetType::ARMOR_WITH_NO_MODEL) {
            plan.aimed_armor_pos << target.armor_x_measurement(0, 0), 
                                target.armor_y_measurement(0, 0), 
                                target.armor_z_measurement(0, 0);

            last_shooted_armor_pos = plan.aimed_armor_pos;
            armor_jump = false;

            // 计算云台目标角度（考虑弹道下降）
            Eigen::Vector2d res;
            res = cal_gimbal_target(plan.aimed_armor_pos, bullet_speed,
                                attitude_yaw, attitude_pitch, R_world2imu);

            plan.target_yaw = res(0, 0);
            plan.target_pitch = res(1, 0);

            // 计算目标角速度
            res = cal_target_speed(target.armor_x_state, target.armor_y_state, target.armor_z_state);

            plan.target_yaw_speed = res(0, 0);
            plan.target_pitch_speed = res(1, 0);

            plan.target_yaw_acc = 0;
            plan.target_pitch_acc = 0;

            plan.target_distance = distance_3D(plan.aimed_armor_pos);
            plan.fire_enable = 2;

            if (debug)
                std::cout << "[predictor] target: armor with no model" << std::endl;
        }
        
        // === 策略2: 装甲板模型预测 ===
        // 基于装甲板运动轨迹进行预测，适用于低速运动
        else if (plan.aimed_target_type == AimedTargetType::ARMOR_WITH_ARMOR_MODEL) {
            Pos3D hit_pos;
            hit_pos << target.armor_x_state(0, 0), target.armor_y_state(0, 0), target.armor_z_state(0, 0);
            
            // 计算弹丸飞行时间
            double fly_time = cal_fly_time(hit_pos, bullet_speed, consider_air_resistence);
            double process_latency = duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - tp).count() / 1e6;   
            double total_delay = process_latency + comm_latency + fly_time;

            // 根据装甲板速度预测未来位置
            plan.aimed_armor_pos << hit_pos(0, 0) + total_delay * target.armor_x_state(1, 0), 
                            hit_pos(1, 0) + total_delay * target.armor_y_state(1, 0), 
                            hit_pos(2, 0) + total_delay * target.armor_z_state(1, 0);

            Pos3D shooted_armor_pos;
            shooted_armor_pos << hit_pos(0, 0) + (total_delay + continue_shoot_latency) * target.armor_x_state(1, 0), 
                            hit_pos(1, 0) + (total_delay + continue_shoot_latency) * target.armor_y_state(1, 0), 
                            hit_pos(2, 0) + (total_delay + continue_shoot_latency) * target.armor_z_state(1, 0);

            last_shooted_armor_pos = shooted_armor_pos;
            armor_jump = false;
            
            // 计算云台控制参数
            Eigen::Vector2d res;
            res = cal_gimbal_target(plan.aimed_armor_pos, bullet_speed,
                                attitude_yaw, attitude_pitch,R_world2imu);

            plan.target_yaw = res(0, 0);
            plan.target_pitch = res(1, 0);

            res = cal_target_speed(target.armor_x_state, target.armor_y_state, target.armor_z_state);

            plan.target_yaw_speed = res(0, 0);
            plan.target_pitch_speed = res(1, 0);

            plan.target_yaw_acc = 0;
            plan.target_pitch_acc = 0;

            plan.target_distance = distance_3D(plan.aimed_armor_pos);
            plan.fire_enable = 2;

            if (debug)
                std::cout << "[predictor] target: armor with armor model" << std::endl;
        }
        
        // === 策略3: 整车模型预测装甲板位置 ===
        // 基于整车运动模型预测装甲板位置，使用MPC优化云台轨迹
        else if (plan.aimed_target_type == AimedTargetType::ARMOR_WITH_VEHICLE_MODEL) {
            // 计算平均半径用于初始飞行时间估计
            double r_average = (target.tracked_state(8, 0) + target.tracked_state(9, 0) + target.tracked_state(8, 0)) / 2;
            Pos3D hit_pos_average;
            hit_pos_average << target.tracked_state(2, 0), target.tracked_state(0, 0), target.tracked_state(4, 0);
            double center_yaw = pw_to_yaw(hit_pos_average);
            hit_pos_average(0, 0) += sin(center_yaw) * r_average;
            hit_pos_average(1, 0) += cos(center_yaw) * r_average;
            
            double fly_time = cal_fly_time(hit_pos_average, bullet_speed, consider_air_resistence);
            double process_latency = duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - tp).count() / 1e6;   
            double total_delay = process_latency + comm_latency + fly_time;

            int armor_index;
            // 预测最近的装甲板位置
            plan.aimed_armor_pos = predict_closest_armor(target, total_delay, armor_index);

            // 计算初始偏航角偏移
            double yaw0 = cal_gimbal_target(plan.aimed_armor_pos, bullet_speed,
                                            attitude_yaw, attitude_pitch, R_world2imu)(0, 0);

            // 生成MPC参考轨迹
            Trajectory traj;
            traj = get_trajectory(target, total_delay, yaw0, bullet_speed, attitude_yaw, attitude_pitch, R_world2imu);

            target_yaw_raw = traj(0, HALF_HORIZON) + yaw0;
            target_pitch_raw = traj(2, HALF_HORIZON);

            // === 求解偏航轴MPC ===
            Eigen::VectorXd x0(2);
            x0 << traj(0, 0), traj(1, 0);
            tiny_set_x0(yaw_solver_, x0);

            yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
            tiny_solve(yaw_solver_);

            // === 求解俯仰轴MPC ===
            x0 << traj(2, 0), traj(3, 0);
            tiny_set_x0(pitch_solver_, x0);

            pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
            tiny_solve(pitch_solver_);

            // 提取MPC优化结果
            plan.target_yaw = yaw_solver_->work->x(0, HALF_HORIZON) + yaw0;
            plan.target_yaw_speed = yaw_solver_->work->x(1, HALF_HORIZON);
            plan.target_yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

            plan.target_pitch = pitch_solver_->work->x(0, HALF_HORIZON);
            plan.target_pitch_speed = pitch_solver_->work->x(1, HALF_HORIZON);
            plan.target_pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

            Pos3D shooted_armor_pos = predict_closest_armor(target, total_delay + continue_shoot_latency, armor_index);
            // === 装甲板切换检测 ===
            if (distance_3D(last_shooted_armor_pos - shooted_armor_pos) > same_position_threshold) {
                armor_jump_tp = std::chrono::high_resolution_clock::now();
            }

            double dt_since_jump = duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - armor_jump_tp).count() / 1e6;
            // std::cout << dt_since_jump << std::endl;

            if (dt_since_jump > armor_jump_interval) {
                armor_jump = false;
            }
            else {
                armor_jump = true;
            }

            last_shooted_armor_pos = shooted_armor_pos; 

            // if (a % 10 == 0) {
            //     LOGT_S();

            //     for (int i=0; i != HORIZON; i++) {
            //         std::cout << traj(0, i) + yaw0 << std::endl;
            //     }

            //     for (int i=0; i != HORIZON; i++) {
            //         std::cout << (yaw_solver_->work->x(0, i) + yaw0) << std::endl;
            //     }
            // }
            // a++;

            // LOGT_S();

            // std::cout << traj(0, HALF_HORIZON + shoot_offset) + yaw0 << std::endl;
            // std::cout << (yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset) + yaw0) << std::endl;

            // std::cout << target_yaw_raw << std::endl;
            // std::cout << plan.target_yaw << std::endl;
            // std::cout << armor_jump << std::endl;

            // === 射击决策 ===
            // 基于轨迹跟踪精度决定是否射击
            if (!armor_jump)
                plan.fire_enable = std::hypot(traj(0, HALF_HORIZON + shoot_offset) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset),
                    traj(2, HALF_HORIZON + shoot_offset) - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset)) < same_trace_threshold;
            else
                plan.fire_enable = 0;  // 装甲板切换期间禁止射击

            // std::cout << plan.fire_enable << std::endl;

            plan.target_distance = distance_3D(plan.aimed_armor_pos);

            if (debug)
                std::cout << "[predictor] target: armor with vehicle model" << std::endl;
        }

        // === 策略4: 整车模型瞄准车辆中心 ===
        // 瞄准车辆旋转中心，适用于高速旋转目标，预测发射窗口
        else if (plan.aimed_target_type == AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL) {
            // 计算平均装甲板位置用于飞行时间估计
            double r_average = (target.tracked_state(8, 0) + target.tracked_state(9, 0) + target.tracked_state(8, 0)) / 2;
            Pos3D hit_pos_average;
            hit_pos_average << target.tracked_state(2, 0), target.tracked_state(0, 0), target.tracked_state(4, 0);
            double center_yaw_average = pw_to_yaw(hit_pos_average);
            hit_pos_average(0, 0) += sin(center_yaw_average) * r_average;
            hit_pos_average(1, 0) += cos(center_yaw_average) * r_average;
            
            double fly_time = cal_fly_time(hit_pos_average, bullet_speed, consider_air_resistence);

            double process_latency = duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - tp).count() / 1e6;   
            double total_delay = process_latency + comm_latency + fly_time;

            // 估计预瞄准的装甲板半径
            double next_predicted_yaw = target.tracked_state(6, 0) + target.tracked_state(7, 0) * (total_delay - comm_latency + single_shoot_latency);
            int next_aimed_armor_index = 0;
            for (int i = 0; i != 2; i++) {
                double predicted_armor_yaw = next_predicted_yaw + i * 1.57;

                predicted_armor_yaw = mathutils::limit_rad(predicted_armor_yaw);

                if (predicted_armor_yaw > M_PI_2) predicted_armor_yaw -= M_PI;
                if (predicted_armor_yaw <= -M_PI_2) predicted_armor_yaw += M_PI;

                if ((target.tracked_state(7, 0) > 0 && predicted_armor_yaw < 0) || (target.tracked_state(7, 0) < 0 && predicted_armor_yaw > 0) ||
                    (target.tracked_state(7, 0) == 0)) {
                    next_aimed_armor_index = i;
                    break;
                }

            }

            // 重新计算延时
            double next_r = next_aimed_armor_index == 0 ? target.tracked_state(8, 0) : target.tracked_state(9, 0) + target.tracked_state(8, 0);
            Pos3D hit_pos;
            hit_pos << target.tracked_state(2, 0), target.tracked_state(0, 0), target.tracked_state(4, 0);
            double center_yaw = pw_to_yaw(hit_pos);
            hit_pos(0, 0) += sin(center_yaw) * next_r;
            hit_pos(1, 0) += cos(center_yaw) * next_r;

            // std::cout << "bullet speed: " << bullet_speed << std::endl;
            
            fly_time = cal_fly_time(hit_pos, bullet_speed, consider_air_resistence);

            process_latency = duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - tp).count() / 1e6;   
            total_delay = process_latency + comm_latency + fly_time;

            // std::cout << "total_delay: " << total_delay << std::endl;
            // std::cout << "comm_latency: " << comm_latency << std::endl;
            // std::cout << "fly_time: " << fly_time << std::endl;
            // std::cout << "process_latency: " << process_latency << std::endl;

            // 预测车辆中心位置
            Pos3D aimed_center_pos;
            aimed_center_pos << target.tracked_state(2, 0) + total_delay * target.tracked_state(3, 0), 
                                target.tracked_state(0, 0) + total_delay * target.tracked_state(1, 0), 
                                target.tracked_state(4, 0) + total_delay * target.tracked_state(5, 0);

            // 计算瞄准方向
            double aimed_direction = atan(aimed_center_pos(0, 0) / aimed_center_pos(1, 0));

            // 构造虚拟装甲板状态用于计算瞄准点
            Eigen::Matrix<double, 11, 1> aimed_state;
            if (next_aimed_armor_index == 0) {
                aimed_state << aimed_center_pos(1, 0), 0, aimed_center_pos(0, 0), 0, 
                aimed_center_pos(2, 0), 0, aimed_direction, 0, target.tracked_state(8, 0), 0, 0;
            }
            else {
                aimed_state << aimed_center_pos(1, 0), 0, aimed_center_pos(0, 0), 0, 
                aimed_center_pos(2, 0), 0, aimed_direction, 0, target.tracked_state(8, 0) + target.tracked_state(9, 0), 0, 0;
            }
            Eigen::Matrix<double, 4, 1> aimed_measurement = whole_state_2_measurement(aimed_state);
            plan.aimed_armor_pos << aimed_measurement(1, 0), aimed_measurement(0, 0), aimed_measurement(2, 0);

            Pos3D shooted_center_pos;
            shooted_center_pos << target.tracked_state(2, 0) + (total_delay - comm_latency + single_shoot_latency) * target.tracked_state(3, 0), 
                                target.tracked_state(0, 0) + (total_delay - comm_latency + single_shoot_latency) * target.tracked_state(1, 0), 
                                target.tracked_state(4, 0) + (total_delay - comm_latency + single_shoot_latency) * target.tracked_state(5, 0);

            double shooted_direction = atan(shooted_center_pos(0, 0) / shooted_center_pos(1, 0));

            Eigen::Matrix<double, 11, 1> shooted_state;
            if (next_aimed_armor_index == 0) {
                shooted_state << shooted_center_pos(1, 0), 0, shooted_center_pos(0, 0), 0, 
                shooted_center_pos(2, 0), 0, shooted_direction, 0, target.tracked_state(8, 0), 0, 0;
            }
            else {
                shooted_state << shooted_center_pos(1, 0), 0, shooted_center_pos(0, 0), 0, 
                shooted_center_pos(2, 0), 0, shooted_direction, 0, target.tracked_state(8, 0) + target.tracked_state(9, 0), 0, 0;
            }
            Eigen::Matrix<double, 4, 1> shooted_measurement = whole_state_2_measurement(shooted_state);

            Pos3D shooted_armor_pos;
            shooted_armor_pos << shooted_measurement(1, 0), shooted_measurement(0, 0), shooted_measurement(2, 0);
            
            last_shooted_armor_pos = shooted_armor_pos;
            armor_jump = false;
            
            // 计算云台控制参数
            Eigen::Vector2d res;
            res = cal_gimbal_target(plan.aimed_armor_pos, bullet_speed,
                                attitude_yaw, attitude_pitch, R_world2imu);

            plan.target_yaw = res(0, 0);
            plan.target_pitch = res(1, 0);

            // 使用车辆中心速度计算角速度
            Eigen::Matrix<double, 2, 1> x_state;
            x_state << target.tracked_state(2, 0), target.tracked_state(3, 0);

            Eigen::Matrix<double, 2, 1> y_state;
            y_state << target.tracked_state(0, 0), target.tracked_state(1, 0);

            Eigen::Matrix<double, 2, 1> z_state;
            z_state << target.tracked_state(4, 0), target.tracked_state(5, 0);

            res = cal_target_speed(x_state, y_state, z_state);

            plan.target_yaw_speed = res(0, 0);
            plan.target_pitch_speed = res(1, 0);

            plan.target_yaw_acc = 0;
            plan.target_pitch_acc = 0;

            plan.target_distance = distance_3D(aimed_center_pos) - target.tracked_state(8, 0);

            // === 射击决策 ===
            // 根据预测装甲板偏航角是否处于发射窗口判断是否发射
            double predicted_yaw = target.tracked_state(6, 0) + target.tracked_state(7, 0) * (total_delay - comm_latency + single_shoot_latency);
            int aimed_armor_index = -1;
            for (int i = 0; i != 4; i++) {
                double predicted_armor_yaw = predicted_yaw + i * 1.57;

                predicted_armor_yaw = mathutils::limit_rad(predicted_armor_yaw);

                if (predicted_armor_yaw > M_PI_2) predicted_armor_yaw -= M_PI;
                if (predicted_armor_yaw <= -M_PI_2) predicted_armor_yaw += M_PI;

                if (abs(aimed_direction - predicted_armor_yaw) < fire_threshold) {
                    aimed_armor_index = i;
                    break;
                }

            }

            #if ASSESSMENT_CRITERIA
                if (abs(target.tracked_state(3, 0)) > 0.9) {
                    vx_constant_counter++;

                    if (vx_constant_counter > 20) {
                        if (duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - fire_enable_tp).count() / 1e6 > shoot_interval) {
                            if (aimed_armor_index != -1){
                                plan.fire_enable = 3;
                                fire_enable_tp = std::chrono::high_resolution_clock::now();
                            }
                            else {
                                plan.fire_enable = 0;
                            }
                        }
                        else {
                            plan.fire_enable = 0;
                        }
                    }
                    else {
                        plan.fire_enable = 0;
                    }
                    
                }
                else {
                    plan.fire_enable = 0;
                    vx_constant_counter = 0;
                }
            #else
                if (duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - fire_enable_tp).count() / 1e6 > shoot_interval) {
                    if (aimed_armor_index != -1){
                        plan.fire_enable = 3;
                        fire_enable_tp = std::chrono::high_resolution_clock::now();
                    }
                    else {
                        plan.fire_enable = 0;
                    }
                }
                else {
                    plan.fire_enable = 0;
                }
            #endif

            if (debug)
                std::cout << "[predictor] target: vehicle center with vehicle model" << std::endl;

        }

        return plan;
    }

    /**
     * @brief 整车状态转换为观测值
     * @param x 整车状态向量 [y, vy, x, vx, z, vz, yaw, vyaw, r]
     * @return 观测向量 [ya, xa, z, yaw] - 装甲板位置和偏航角
     * @details 从整车中心状态计算当前装甲板的观测值
     *          ya = yc - r * cos(yaw)  // 装甲板Y坐标 = 车辆中心Y坐标 - 半径*cos(偏航角)
     *          xa = xc - r * sin(yaw)  // 装甲板X坐标 = 车辆中心X坐标 - 半径*sin(偏航角)
     */
    Eigen::Matrix<double, 4, 1> Planner::whole_state_2_measurement(const Eigen::Matrix<double, 11, 1> &x) 
    {
        double ya = x(0, 0) - x(8, 0) * cos(x(6, 0));
        double xa = x(2, 0) - x(8, 0) * sin(x(6, 0));

        return Eigen::Matrix<double, 4, 1>(ya, xa, x(4, 0), x(6, 0));
    }

    /**
     * @brief 预测最近的装甲板位置
     * @param target 目标跟踪状态
     * @param delay 预测时间延迟 (秒)
     * @return 预测的装甲板位置 [x, y, z]
     * @details 基于整车模型预测四个装甲板位置，返回距离最近的一个
     *          假设装甲板按90度间隔分布在车辆周围
     *          考虑不同装甲板对可能有不同的半径和高度差
     */
    Eigen::Matrix<double, 3, 1> Planner::predict_closest_armor(const Target &target, const double delay, int &armor_index)
    {
        auto &x = target.tracked_state;

        // 预测车辆中心位置
        Pos3D predicted_center_pos;
        predicted_center_pos << x(2, 0) + x(3, 0) * delay, x(0, 0) + x(1, 0) * delay, x(4, 0) + x(5, 0) * delay;

        // 预测车辆偏航角
        double predicted_yaw = x(6, 0) + x(7, 0) * delay;

        // 计算四个装甲板的预测位置
        std::vector<Pos3D> predicted_armors_pos;
        for (int i = 0; i != 4; i++) {
            if (i == 0 || i == 2) {
                // 第0和第2个装甲板：使用当前跟踪的半径
                Pos3D armor_pos;
                armor_pos << predicted_center_pos(0, 0) + x(8, 0) * sin(predicted_yaw + 1.57 * i), 
                                predicted_center_pos(1, 0) + x(8, 0) * cos(predicted_yaw + 1.57 * i), 
                                predicted_center_pos(2, 0);
                predicted_armors_pos.push_back(armor_pos);
            }
            else {
                // 第1和第3个装甲板：使用另一对装甲板的半径和高度差
                Pos3D armor_pos;
                armor_pos << predicted_center_pos(0, 0) + (x(8, 0) + x(9, 0)) * sin(predicted_yaw + 1.57 * i), 
                                predicted_center_pos(1, 0) + (x(8, 0) + x(9, 0)) * cos(predicted_yaw + 1.57 * i), 
                                predicted_center_pos(2, 0) + x(10, 0);
                predicted_armors_pos.push_back(armor_pos);
            }
        }

        // 寻找距离最近的装甲板
        double min_dist = DBL_MAX;
        int closest_armor_idx;
        for (int i = 0; i != 4; i++) {
            double dist = distance_2D(predicted_armors_pos[i]);

            if (dist < min_dist) {
                min_dist = dist;
                closest_armor_idx = i;
            }
        }

        armor_index = closest_armor_idx;

        Eigen::Matrix<double, 3, 1> res;
        res << predicted_armors_pos[closest_armor_idx];

        return res;
    }

    /**
     * @brief 计算云台目标角度
     * @param aimed_armor_pos 瞄准的装甲板位置
     * @param bullet_speed 弹丸速度
     * @param attitude_yaw 机器人当前偏航角
     * @param attitude_pitch 机器人当前俯仰角
     * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
     * @return [target_yaw, target_pitch] 云台目标角度
     * @details 考虑弹道下降，计算云台应达到的偏航角和俯仰角
     *          弹道补偿：z = z0 - 0.5 * g * t²
     *          通过坐标变换计算IMU坐标系下的角度
     */
    Eigen::Matrix<double, 2, 1> Planner::cal_gimbal_target(Eigen::Matrix<double, 3, 1> aimed_armor_pos,
                                                            const float bullet_speed,
                                                            const double attitude_yaw,
                                                            const double attitude_pitch,
                                                            const Eigen::Matrix3d &R_world2imu) 
    {
        Pos3D pw{aimed_armor_pos(0, 0), aimed_armor_pos(1, 0), aimed_armor_pos(2, 0)};

        // 计算弹丸飞行时间并进行弹道补偿
        double fly_time = cal_fly_time(pw, bullet_speed, consider_air_resistence);
        pw(2, 0) -= 0.5 * g * fly_time * fly_time;  // 重力下降补偿

        // 世界坐标系转换为IMU坐标系
        Pos3D pi = coord_transformer.pw_to_pi(pw, R_world2imu);   

        // 计算IMU坐标系下的目标角度，并叠加机器人当前姿态
        return {atan(pi(0, 0) / pi(2, 0)) + attitude_yaw + yaw_comp, atan(pi(1, 0) / pi(2, 0)) + attitude_pitch + pitch_comp};
    }

    /**
     * @brief 生成MPC参考轨迹
     * @param target 目标跟踪状态
     * @param total_delay 总延迟时间
     * @param yaw0 初始偏航角偏移
     * @param bullet_speed 弹丸速度
     * @param attitude_yaw 机器人偏航角
     * @param attitude_pitch 机器人俯仰角
     * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
     * @return 完整的参考轨迹矩阵
     * @details 生成HORIZON长度的参考轨迹，包含偏航角、偏航角速度、俯仰角、俯仰角速度
     *          使用中心差分法计算角速度：v(k) = [x(k+1) - x(k-1)] / (2*dt)
     */
    Trajectory Planner::get_trajectory(const Target &target, const double total_delay, const double yaw0, const double bullet_speed,
                                        const double attitude_yaw,
                                        const double attitude_pitch, const Eigen::Matrix3d &R_world2imu)
    {
        Trajectory traj;

        // 计算轨迹起始时间点
        double delay_start = total_delay - DT * HALF_HORIZON;
        
        int armor_index;
        // 预计算前两个时刻的位置用于中心差分
        Eigen::Matrix<double, 3, 1> last_pos = predict_closest_armor(target, delay_start - DT, armor_index);
        auto yaw_pitch_last = cal_gimbal_target(last_pos, bullet_speed, attitude_yaw, attitude_pitch, R_world2imu);

        Eigen::Matrix<double, 3, 1> cur_pos = predict_closest_armor(target, delay_start, armor_index);
        auto yaw_pitch_cur = cal_gimbal_target(cur_pos, bullet_speed, attitude_yaw, attitude_pitch, R_world2imu);

        // 生成完整轨迹
        for (int i = 0; i < HORIZON; i++) {
            // 预测下一时刻的位置
            Eigen::Matrix<double, 3, 1> next_pos = predict_closest_armor(target, delay_start + DT * (i+1), armor_index);
            auto yaw_pitch_next = cal_gimbal_target(next_pos, bullet_speed, attitude_yaw, attitude_pitch, R_world2imu);

            // 使用中心差分法计算角速度
            auto yaw_vel = mathutils::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
            auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

            // 构建轨迹点：[yaw_relative, yaw_vel, pitch, pitch_vel]
            traj.col(i) << mathutils::limit_rad(yaw_pitch_cur(0) - yaw0), yaw_vel, yaw_pitch_cur(1), pitch_vel;

            // 更新时间序列
            yaw_pitch_last = yaw_pitch_cur;
            yaw_pitch_cur = yaw_pitch_next;
        }

        return traj;
    }

    /**
     * @brief 设置偏航轴MPC求解器
     * @details 配置偏航轴的MPC问题：
     *          状态空间模型：[position, velocity]
     *          控制输入：acceleration
     *          代价函数：重点惩罚跟踪误差，轻微惩罚控制输入
     *          约束：限制最大角加速度
     */
    void Planner::setup_yaw_solver()
    {
        // 代价函数权重：高度重视位置跟踪，忽略速度跟踪
        Eigen::Matrix<double, 2, 1> Q;
        Q << 9e6, 0;  // 位置权重很大，速度权重为0
        
        // 控制输入权重：较小的值，允许必要的控制
        Eigen::Matrix<double, 1, 1> R;
        R << 1;

        // 离散时间状态空间模型
        Eigen::MatrixXd A{{1, DT}, {0, 1}};  // 状态转移矩阵
        Eigen::MatrixXd B{{0}, {DT}};        // 控制输入矩阵
        Eigen::VectorXd f{{0, 0}};           // 常数项（线性系统为0）
        
        // 初始化MPC求解器
        tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        // 设置约束：状态无约束，控制输入有约束
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);      // 状态下界（无约束）
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);       // 状态上界（无约束）
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);   // 最大负加速度
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);    // 最大正加速度
        tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

        // 设置求解器参数：限制迭代次数以保证实时性
        yaw_solver_->settings->max_iter = 10;
    }

    /**
     * @brief 设置俯仰轴MPC求解器
     * @details 配置俯仰轴的MPC问题，参数设置与偏航轴类似
     *          但使用不同的物理约束（俯仰轴通常有更大的加速度能力）
     */
    void Planner::setup_pitch_solver()
    {
        // 代价函数权重：与偏航轴相同
        Eigen::Matrix<double, 2, 1> Q;
        Q << 9e6, 0;
        Eigen::Matrix<double, 1, 1> R;
        R << 1;

        // 状态空间模型：与偏航轴相同
        Eigen::MatrixXd A{{1, DT}, {0, 1}};
        Eigen::MatrixXd B{{0}, {DT}};
        Eigen::VectorXd f{{0, 0}};
        tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

        // 设置约束：俯仰轴有更大的加速度能力
        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
        tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

        pitch_solver_->settings->max_iter = 10;
    }

    /**
     * @brief 更新瞄准目标类型
     * @param target 目标跟踪状态
     * @param rotation_speed 目标旋转速度
     * @details 根据跟踪状态和旋转速度自动选择最适合的预测策略
     *          状态转换逻辑：
     *          - TRACKING: 根据旋转速度在不同策略间切换
     *          - DETECTING: 固定使用无模型预测
     *          - TEMP_LOST: 根据上次的旋转速度选择策略
     *          - IDLE: 无目标
     */
    void Planner::update_aimed_target_type(const Target &target, double rotation_speed)
    {
        // 根据旋转速度选择不同的云台目标模型
        if (target.predictor_state == TrackingState::TRACKING) {
            if (plan.aimed_target_type == AimedTargetType::NONE || plan.aimed_target_type == AimedTargetType::ARMOR_WITH_NO_MODEL) {
                // 从无目标或无模型状态转换
                if (rotation_speed > medium_rotation_upper_bound) {
                    if (disable_vehicle_center_shoot_mode) {
                        plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                    }
                    else {
                        plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                    }
                }
                else if (rotation_speed > slow_rotation_upper_bound) {
                    if (disable_armor_with_vehicle_shoot_mode) {
                        plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                    }
                    else {
                        plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                    }
                }
                else {
                    plan.aimed_target_type = AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
                }
            }
            else if (plan.aimed_target_type == AimedTargetType::ARMOR_WITH_ARMOR_MODEL) {
                // 从装甲板模型状态转换：只能向上升级
                if (rotation_speed > slow_rotation_upper_bound) {
                    if (disable_armor_with_vehicle_shoot_mode) {
                        plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                    }
                    else {
                        plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                    }
                }
            }
            else if (plan.aimed_target_type == AimedTargetType::ARMOR_WITH_VEHICLE_MODEL) {
                // 从整车模型状态转换：可以向上或向下
                if (rotation_speed > medium_rotation_upper_bound) {
                    if (disable_vehicle_center_shoot_mode) {
                        plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                    }
                    else {
                        plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                    }
                }
                else if (rotation_speed < medium_rotation_lower_bound) {
                    plan.aimed_target_type = AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
                }
            }
            else if (plan.aimed_target_type == AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL) {
                // 从车辆中心模型状态转换：只能向下降级
                if (rotation_speed < fast_rotation_lower_bound) {
                    if (disable_armor_with_vehicle_shoot_mode) {
                        plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                    }
                    else {
                        plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                    }
                }
            }

            if (target.vehicle_model_trust == false && 
                (plan.aimed_target_type == AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL 
                    || plan.aimed_target_type == AimedTargetType::ARMOR_WITH_VEHICLE_MODEL)) {
                plan.aimed_target_type = AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
            }
        }
        else if (target.predictor_state == TrackingState::DETECTING) {
            // 检测阶段：使用最简单的无模型预测
            plan.aimed_target_type = AimedTargetType::ARMOR_WITH_NO_MODEL;
        }
        else if (target.predictor_state == TrackingState::TEMP_LOST) {
            // 暂时丢失阶段：根据上次的旋转速度选择合适策略
            if (rotation_speed > medium_rotation_upper_bound) {
                if (disable_vehicle_center_shoot_mode) {
                    plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                }
                else {
                    plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                }
            }
            else if (rotation_speed > slow_rotation_upper_bound) {
                if (disable_armor_with_vehicle_shoot_mode) {
                    plan.aimed_target_type = AimedTargetType::VEHICLE_CENTER_WITH_VEHICLE_MODEL;
                }
                else {
                    plan.aimed_target_type = AimedTargetType::ARMOR_WITH_VEHICLE_MODEL;
                }
            }
            else {
                plan.aimed_target_type = AimedTargetType::ARMOR_WITH_ARMOR_MODEL;
            }
        }
        else {
            // 空闲状态：无目标
            plan.aimed_target_type = AimedTargetType::NONE;
        }
    }

    /**
     * @brief 计算目标角速度
     * @param x_state X坐标状态 [x, vx]
     * @param y_state Y坐标状态 [y, vy]
     * @param z_state Z坐标状态 [z, vz]
     * @return [yaw_speed, pitch_speed] 目标角速度
     * @details 从直角坐标速度计算对应的角速度
     *          偏航角速度 = (r × v) / |r|²，其中r为位置向量，v为速度向量
     *          俯仰角速度计算类似，使用不同的坐标分量
     */
    Eigen::Matrix<double, 2, 1> Planner::cal_target_speed(const Eigen::Matrix<double, 2, 1> &x_state, 
                                                            const Eigen::Matrix<double, 2, 1> &y_state, 
                                                            const Eigen::Matrix<double, 2, 1> &z_state)
    {
        double target_yaw_speed;
        double target_pitch_speed;

        // === 计算偏航角速度 ===
        // 使用X-Y平面的位置和速度向量
        Eigen::Vector2d r_vec(x_state(0, 0), y_state(0, 0));  // 位置向量
        Eigen::Vector2d v_vec(x_state(1, 0), y_state(1, 0));  // 速度向量

        double r2 = r_vec.squaredNorm();  // 位置向量的模长平方
        if (r2 < 1e-6) {
            target_yaw_speed = 0.0;
        } else {
            // 叉积计算：ω = (r × v) / |r|²
            double cross = r_vec.x() * v_vec.y() - r_vec.y() * v_vec.x();
            target_yaw_speed = (cross / r2);
        }

        // === 计算俯仰角速度 ===
        // 使用Y-Z平面的位置和速度向量
        r_vec << y_state(0, 0), z_state(0, 0);
        v_vec << y_state(1, 0), z_state(1, 0);

        r2 = r_vec.squaredNorm();
        if (r2 < 1e-6) {
            target_pitch_speed = 0.0;
        } else {
            double cross = r_vec.x() * v_vec.y() - r_vec.y() * v_vec.x();
            target_pitch_speed = (cross / r2);
        }

        return {target_yaw_speed, target_pitch_speed};
    }

}
