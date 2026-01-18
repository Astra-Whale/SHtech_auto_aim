/**
 * @file Tracker.cpp
 * @brief 目标跟踪器实现 - 多模型自适应跟踪算法的具体实现
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 实现功能：
 * 1. 多滤波器初始化和参数设置
 * 2. 跟踪状态机逻辑
 * 3. 模型自适应切换策略
 * 4. 异常检测和恢复机制
 */

#include "Tracker.hpp"

namespace predict 
{
    int ab = 0;

    /**
     * @brief 构造函数 - 初始化跟踪器的所有组件
     * @param debug_ 调试模式标志
     * @param adjust_ 参数调整模式标志
     * @details 设置初始状态，初始化滤波器，配置参数调整界面（如果需要）
     */
    Tracker::Tracker(bool debug_, bool adjust_)
    : last_tp(std::chrono::high_resolution_clock::now()),
      detecting_counter(0),
      temp_lost_counter(0),
      yaw_speed_diverge_counter(0),
      rotate_counter(0),
      debug(debug_),
      adjust(adjust_)
    {
        // 如果开启参数调整模式，初始化调整界面
        if (adjust) {
            parameter_adjustor_init();
        }

        // 初始化目标状态
        target_init();

        // 初始化装甲板模型的卡尔曼滤波器组
        armor_state_kf_init();
        
        // 初始化整车模型的扩展卡尔曼滤波器
        whole_state_ekf_init();
    }

    /**
     * @brief 重置目标跟踪 - 用新检测到的目标重新初始化跟踪器
     * @param measurement 新的观测值 [y, x, z, yaw]
     * @param tp 当前时间戳
     * @return 重置后的目标状态引用
     * @details 重置所有滤波器状态，进入检测阶段
     */
    const Target& Tracker::reset_target(const Eigen::Matrix<double, 4, 1> &measurement, TP &tp)
    {
        // 保存新的观测值
        target.tracked_measurement = measurement;

        // 重置所有滤波器到初始状态
        reset_whole_state_ekf();
        reset_yaw_kf();
        reset_armor_x_kf();
        reset_armor_y_kf();
        reset_armor_z_kf();

        // 进入检测阶段
        detecting_counter++;
        target.predictor_state = TrackingState::DETECTING;

        // 初始使用装甲板模型
        target.updating_model_type = UpdatingModelType::ARMOR_MODEL;

        rotate_counter = 0;
        target.vehicle_model_trust = false;

        return target;
    }

    /**
     * @brief 获取当前目标状态
     * @return 目标状态结构体的常引用
     */
    const Target& Tracker::get_target()
    {
        return target;
    }

    /**
     * @brief 执行目标跟踪更新 - 核心跟踪算法
     * @param measurement 当前观测值 [y, x, z, yaw]
     * @param secondary_measurement 当前备选观测值 [y, x, z, yaw]
     * @param same_id_armor_count 检测到的同ID装甲板数量
     * @param tp 当前时间戳
     * @param attitude_yaw 机器人当前姿态偏航角
     * @return 更新后的目标状态引用
     * @details 执行完整的跟踪流程：预测、更新、模型选择、异常检测
     */
    const Target& Tracker::track(const Eigen::Matrix<double, 4, 1> &measurement, const Eigen::Matrix<double, 4, 1> &secondary_measurement, const int same_id_armor_count,
                                    const TP &tp, const double attitude_yaw)
    {
        // 注意：以下逻辑都是针对单个车辆的跟踪

        // 计算时间间隔
        dt = duration_cast<microseconds>(tp - last_tp).count() / 1e6;

        // 计算位置变化用于判断装甲板是否跳变
        Eigen::Matrix<double, 3, 1> measured_pw(target.tracked_measurement(1, 0), target.tracked_measurement(0, 0), target.tracked_measurement(2, 0));
        Eigen::Matrix<double, 3, 1> tracked_pw(measurement(1, 0), measurement(0, 0), measurement(2, 0));
        double min_position_diff = (measured_pw - tracked_pw).norm();
        
        // 更新当前观测值
        // TODO: add secondary measurement and limit 
        target.tracked_measurement = measurement;

        target.tracked_measurement(3, 0) = mathutils::limit_rad(target.tracked_measurement(3, 0));

        // === 扩展卡尔曼滤波预测步骤 ===
        target.tracked_state = whole_state_ekf.predict();

        if (debug) {
            cout << "dt: " << dt << std::endl;
            cout << "[predict] predict" << std::endl;
        }

        // 更新跟踪器状态机
        update_tracker_state(same_id_armor_count);

        // === 偏航角卡尔曼滤波更新 ===
        // 用于模型选择
        if (adjust) {
            update_parameter();
        }

        if (same_id_armor_count) {
            if (min_position_diff < same_position_threshold) {
                target.yaw_measurement << target.tracked_measurement(3, 0);
                target.yaw_state = yaw_kf.update(target.yaw_measurement); 
            }
            else {
                //装甲板跳变，重置滤波器
                reset_yaw_kf();

                if (debug)
                    std::cout << "[predict] yaw kf reset" << std::endl;
            }
        }

        // === 模型选择策略 ===
        tracker_model_select();

        target.updating_model_type = UpdatingModelType::VEHICLE_MODEL;
        
        // === 装甲板模型更新 (x, y, z坐标) ===
        // 使用白噪声运动模型
        if (same_id_armor_count) {
            if (target.updating_model_type != UpdatingModelType::VEHICLE_MODEL) {
                if (min_position_diff < same_position_threshold) {
                    target.armor_y_measurement << target.tracked_measurement(0, 0);
                    target.armor_y_state = armor_y_kf.update(target.armor_y_measurement); 

                    target.armor_x_measurement << target.tracked_measurement(1, 0);
                    target.armor_x_state = armor_x_kf.update(target.armor_x_measurement); 

                    target.armor_z_measurement << target.tracked_measurement(2, 0);
                    target.armor_z_state = armor_z_kf.update(target.armor_z_measurement); 

                    if (debug)
                        cout << "[predictor] kf update" << endl;
                }
                else {
                    // 位置跳变，重置装甲板模型滤波器
                    reset_armor_y_kf();
                    reset_armor_x_kf();
                    reset_armor_z_kf();

                    if (debug)
                        std::cout << "[predict] armor state kf reset" << std::endl;
                }
            }
            else {
                // 仅使用整车模型时，重置装甲板模型滤波器
                reset_armor_y_kf();
                reset_armor_x_kf();
                reset_armor_z_kf();

                if (debug)
                    std::cout << "[predict] vehicle model update only" << std::endl;
            }
        }

        // === 整车模型更新 ===
        if (same_id_armor_count) {
            if (target.updating_model_type != UpdatingModelType::ARMOR_MODEL) {
                if (min_position_diff > same_position_threshold) {
                    // ab++;
                    ab--;

                    if (ab >= 4) {
                        ab = 0;
                    }
                    else if (ab < 0) {
                        ab = 3;
                    }
                }

                if (same_id_armor_count == 1) {
                    // cout << "[predict] 1" << std::endl;

                    int id = match_armor_id(target.tracked_measurement);
                    // int id = ab;
                    // cout << "[predict] id: " << id << "ab: " << ab << std::endl;

                    target.tracked_state = whole_state_ekf.update(target.tracked_measurement, id);
                }
                else if (same_id_armor_count == 2) {
                    // cout << "[predict] 2" << std::endl;
                    int id = match_armor_id(target.tracked_measurement);
                    // int id = ab;
                    // cout << "[predict] id: " << id << "ab: " << ab << std::endl;
                    target.tracked_state = whole_state_ekf.update(target.tracked_measurement, id);

                    // id = match_armor_id(target.tracked_measurement);

                    id = match_armor_id(secondary_measurement);
                    // cout << "[predict] id: " << id << std::endl;

                    target.tracked_state = whole_state_ekf.update(secondary_measurement, id);
                }

                if (min_position_diff > same_position_threshold) {
                    // 装甲板跳变处理
                    // 切换装甲板计数器          
                    target.ab_counter = 1 - target.ab_counter;

                    if (rotate_counter < least_rotate_count) {
                        rotate_counter++;
                    }

                    if (rotate_counter < least_rotate_count) {
                        target.vehicle_model_trust = false;
                    }
                    else {
                        target.vehicle_model_trust = true;
                    }

                    if (debug)
                        cout << "[predict] armor jump" << std::endl;
                }


                if (debug)
                    cout << "[predict] ekf update" << std::endl;

                // 限制旋转半径在合理范围内
                radium_limit();

                // // 检查EKF是否发散
                // if (check_ekf_divergence(attitude_yaw)) {
                //     reset_whole_state_ekf();

                //     if (debug)
                //         cout << "[predict] vehicle model converge" << endl;
                // }
            }
            else {
                // 仅使用装甲板模型时，重置整车模型
                reset_whole_state_ekf();

                if (debug)
                    cout << "[predict] armor model update only" << endl;
            }
        }
        else {
            if (debug)
                std::cout << "[predict] no same id armor" << std::endl;
        }

        // 更新时间戳
        last_tp = tp;

        return target;
    }

    Eigen::Vector3d Tracker::h_armor_xyz(const Eigen::VectorXd & x, int id)
    {
        auto angle = mathutils::limit_rad(x[6] + id * M_PI_2);
        auto use_l_h = id == 1 || id == 3;

        auto r = (use_l_h) ? x[8] + x[9] : x[8];
        auto armor_x = x[0] - r * std::cos(angle);
        auto armor_y = x[2] - r * std::sin(angle);
        auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

        return {armor_x, armor_y, armor_z};
    }

    std::vector<Eigen::Vector4d> Tracker::armor_xyza_list()
    {
        std::vector<Eigen::Vector4d> _armor_xyza_list;

        for (int i = 0; i < 4; i++) {
            auto angle = mathutils::limit_rad(target.tracked_state(6, 0) + i * M_PI_2);
            Eigen::Vector3d xyz = h_armor_xyz(target.tracked_state, i);
            _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
        }
        return _armor_xyza_list;
    }

    int Tracker::match_armor_id(const Eigen::Matrix<double, 4, 1> &measurement)
    {
        int id;
        auto min_angle_error = 1e10;
        const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
      
        std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
        for (int i = 0; i < 4; i++) {
          xyza_i_list.push_back({xyza_list[i], i});
        }
      
        std::sort(
          xyza_i_list.begin(), xyza_i_list.end(),
          [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
            Eigen::Vector3d ypd1 = mathutils::xyz2ypd(a.first.head(3));
            Eigen::Vector3d ypd2 = mathutils::xyz2ypd(b.first.head(3));
            return ypd1[2] < ypd2[2];
          });

        Eigen::Vector3d xyz_measurement;
        xyz_measurement << measurement(0, 0), measurement(1, 0), measurement(2, 0);
        Eigen::Vector3d ypd_measurement = mathutils::xyz2ypd(xyz_measurement);
      
        // 取前3个distance最小的装甲板
        for (int i = 0; i < 3; i++) {
          const auto & xyza = xyza_i_list[i].first;
          Eigen::Vector3d ypd = mathutils::xyz2ypd(xyza.head(3));
          auto angle_error = std::abs(mathutils::limit_rad(measurement(3, 0) - xyza[3])) +
                             std::abs(mathutils::limit_rad(ypd_measurement[0] - ypd[0]));

        // auto angle_error = std::abs(mathutils::limit_rad(ypd_measurement[0] - ypd[0]));
      
          if (std::abs(angle_error) < std::abs(min_angle_error)) {
            id = xyza_i_list[i].second;
            min_angle_error = angle_error;
          }
        }

        return id;
    }

    /**
     * @brief 获取当前跟踪状态
     * @return 跟踪器状态枚举值
     */
    TrackingState Tracker::get_tracker_state()
    {
        return target.predictor_state;
    }

    /**
     * @brief 初始化目标状态参数
     * @details 设置目标的初始状态和默认参数
     */
    void Tracker::target_init() 
    {
        target.predictor_state = TrackingState::IDLE;
        target.another_r = 0.26;  // 另一对装甲板的默认半径 (米)
        target.ab_counter = 0;    // 装甲板切换计数器
        target.updating_model_type = UpdatingModelType::ARMOR_MODEL;
        target.vehicle_model_trust = false;
    }

    /**
     * @brief 初始化装甲板模型卡尔曼滤波器组
     * @details 设置四个独立的2D卡尔曼滤波器：偏航角、X坐标、Y坐标、Z坐标
     *          每个滤波器使用位置-速度模型，状态为 [位置, 速度]
     */
    void Tracker::armor_state_kf_init()
    {
        // === 偏航角卡尔曼滤波器初始化 ===
        auto yaw_update_A = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> A;

            A << 1,   dt,
                 0,   1;

            return A;
        };

        auto yaw_update_H = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 2> H;

            H << 1,
                 0;

            return H;
        };

        auto yaw_update_Q = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> Q;
            // 连续白噪声加速度模型的离散化过程噪声协方差
            Q << pow(dt, 4)/4*q_kf_yaw, pow(dt, 3)/2*q_kf_yaw,
                 pow(dt, 3)/2*q_kf_yaw, pow(dt, 2)*q_kf_yaw;
            return Q;
        };

        auto yaw_update_R = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 1> R;

            R << 1.0;//r_yaw;

            return R;
        };

        yaw_kf = filter_2d(yaw_update_A, yaw_update_H, yaw_update_Q, yaw_update_R, 
                           Eigen::Matrix<double, 2, 2>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());

        // === Y坐标卡尔曼滤波器初始化 ===
        auto y_update_A = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> A;


            A << 1,   dt,
                 0,   1;

            return A;
        };

        auto y_update_H = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 2> H;

            H << 1,
                 0;

            return H;
        };

        auto y_update_Q = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> Q;
            Q << pow(dt, 4)/4*q_kf_y, pow(dt, 3)/2*q_kf_y,
                 pow(dt, 3)/2*q_kf_y, pow(dt, 2)*q_kf_y;
            return Q;
        };

        auto y_update_R = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 1> R;

            R << 1.0;//r_rad_coeff;

            return R;
        };

        armor_y_kf = filter_2d(y_update_A, y_update_H, y_update_Q, y_update_R, 
                               Eigen::Matrix<double, 2, 2>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());

        // === X坐标卡尔曼滤波器初始化 ===
        auto x_update_A = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> A;
            A << 1,   dt,
                 0,   1;
            return A;
        };

        auto x_update_H = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 2> H;

            H << 1,
                 0;

            return H;
        };

        auto x_update_Q = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> Q;

            Q << pow(dt, 4)/4*q_kf_x, pow(dt, 3)/2*q_kf_x,
                 pow(dt, 3)/2*q_kf_x, pow(dt, 2)*q_kf_x;

            return Q;
        };

        auto x_update_R = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 1> R;
            R << 1.0;//r_tan_coeff;  // X坐标观测噪声方差
            return R;
        };

        armor_x_kf = filter_2d(x_update_A, x_update_H, x_update_Q, x_update_R, 
                               Eigen::Matrix<double, 2, 2>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());

        // === Z坐标卡尔曼滤波器初始化 ===
        auto z_update_A = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> A;
            A << 1,   dt,
                 0,   1;
            return A;
        };

        auto z_update_H = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 2> H;
            H << 1, 0;
            return H;
        };

        auto z_update_Q = [this](const Eigen::Matrix<double, 2, 1> & x) {
            Eigen::Matrix<double, 2, 2> Q;
            Q << pow(dt, 4)/4*q_kf_z, pow(dt, 3)/2*q_kf_z,
                 pow(dt, 3)/2*q_kf_z, pow(dt, 2)*q_kf_z;
            return Q;
        };

        auto z_update_R = [this](const Eigen::Matrix<double, 1, 1> & z) {
            Eigen::Matrix<double, 1, 1> R;
            R << 1.0;//r_z_coeff;  // Z坐标观测噪声方差
            return R;
        };

        armor_z_kf = filter_2d(z_update_A, z_update_H, z_update_Q, z_update_R, 
                               Eigen::Matrix<double, 2, 2>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());
    }

    /**
     * @brief 初始化整车状态扩展卡尔曼滤波器
     * @details 设置9维状态向量的EKF：[y, vy, x, vx, z, vz, yaw, vyaw, r]
     *          观测向量为4维：[y, x, z, yaw] (装甲板位置和朝向)
     */
    void Tracker::whole_state_ekf_init()
    {
        // === 状态转移函数 ===
        // 状态: y, vy, x, vx, z, vz, yaw(-∞, +∞), vyaw, r, l, h
        // 观测: y, x, z, yaw(-∞, +∞)
        auto f_ = [this](const Eigen::Matrix<double, 11, 1> & x) {
            Eigen::Matrix<double, 11, 1> x_pri = x;
            // 积分更新位置和角度
            x_pri(0, 0) += x(1, 0) * dt;  // y = y + vy * dt
            x_pri(2, 0) += x(3, 0) * dt;  // x = x + vx * dt
            x_pri(4, 0) += x(5, 0) * dt;  // z = z + vz * dt
            x_pri(6, 0) += x(7, 0) * dt;  // yaw = yaw + vyaw * dt
            x_pri(6, 0) = mathutils::limit_rad(x_pri(6, 0)); // 限制yaw在[-π, π]范围内
            return x_pri;
        };

        // === 状态转移雅可比矩阵 ===
        auto cal_F_ = [this](const Eigen::Matrix<double, 11, 1> & x) {
            Eigen::Matrix<double, 11, 11> F;
            F << 1,   dt, 0,   0,   0,   0,   0,   0,   0,  0,  0,
                 0,   1,   0,   0,   0,   0,   0,   0,   0, 0,  0,
                 0,   0,   1,   dt, 0,   0,   0,   0,   0,  0,  0,
                 0,   0,   0,   1,   0,   0,   0,   0,   0, 0,  0,
                 0,   0,   0,   0,   1,   dt, 0,   0,   0,  0,  0,
                 0,   0,   0,   0,   0,   1,   0,   0,   0, 0,  0,
                 0,   0,   0,   0,   0,   0,   1,   dt, 0,  0,  0,
                 0,   0,   0,   0,   0,   0,   0,   1,   0, 0,  0,
                 0,   0,   0,   0,   0,   0,   0,   0,   1, 0,  0,
                 0,   0,   0,   0,   0,   0,   0,   0,   0, 1,  0,
                 0,   0,   0,   0,   0,   0,   0,   0,   0, 0,  1;
            return F;
        };

        // === 观测函数 ===
        // 从车辆中心状态计算装甲板位置
        auto h_ = [](const Eigen::Matrix<double, 11, 1> & x, const int id) {
            Eigen::Matrix<double, 4, 1> z;

            auto angle = mathutils::limit_rad(x(6, 0) + id * M_PI_2);
            auto is_another_r = id == 1 || id == 3;
            auto r = is_another_r ? x(8, 0) + x(9, 0) : x(8, 0);

            z(0, 0) = x(0, 0) - r * cos(angle);  // ya = yc - r * cos(yaw)
            z(1, 0) = x(2, 0) - r * sin(angle);  // xa = xc - r * sin(yaw)
            z(2, 0) = is_another_r ? x(4, 0) + x(10, 0) : x(4, 0);  // za = zc
            z(3, 0) = angle;                            // yaw_a = yaw_c
            return z;
        };

        // === 观测雅可比矩阵 ===
        auto cal_H_ = [](const Eigen::Matrix<double, 11, 1> & x, const int id) {
            auto angle = mathutils::limit_rad(x(6, 0) + id * M_PI_2);

            auto is_another_r = id == 1 || id == 3;
            auto r = is_another_r ? x(8, 0) + x(9, 0) : x(8, 0);

            auto dx_dl = is_another_r ? -cos(angle) : 0.0;
            auto dy_dl = is_another_r ? -sin(angle) : 0.0;
            auto dz_dh = is_another_r ? 1.0 : 0.0; 

            Eigen::Matrix<double, 4, 11> H;
            H << 1,   0,   0,   0,   0,   0,   r*sin(angle), 0,   -cos(angle),  dx_dl,  0,
                 0,   0,   1,   0,   0,   0,   -r*cos(angle),0,   -sin(angle),  dy_dl,  0,
                 0,   0,   0,   0,   1,   0,   0,                    0,   0,    0,  dz_dh,
                 0,   0,   0,   0,   0,   0,   1,                    0,   0,    0,  0;
            return H;
        };

        // === 过程噪声协方差矩阵 ===
        auto update_Q_ = [this](const Eigen::Matrix<double, 11, 1> & x) {
            Eigen::Matrix<double, 11, 11> Q;
            // 连续白噪声加速度模型
            double q_x_x = pow(dt, 4) / 4 * p_coord, q_x_vx = pow(dt, 3) / 2 * p_coord, q_vx_vx = pow(dt, 2) * p_coord;
            double q_y_y = pow(dt, 4) / 4 * p_yaw, q_y_vy = pow(dt, 3) / 2 * p_yaw, q_vy_vy = pow(dt, 2) * p_yaw;

            Q << q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0, 0,  0,  
                 q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0, 0,  0,  
                 0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0, 0,  0,  
                 0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0, 0,  0,  
                 0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0, 0,  0,  
                 0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0, 0,  0,  
                 0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0, 0,  0,  
                 0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0, 0,  0,  
                 0,      0,      0,      0,      0,      0,      0,      0,      0e-4, 0,  0,  
                 0,      0,      0,      0,      0,      0,      0,      0,      0, 0e-4,  0,  
                 0,      0,      0,      0,      0,      0,      0,      0,      0, 0,  0e-4;
            return Q;
        };

        // === 观测噪声协方差矩阵 R 更新 (基于球坐标系 YPD + Yaw 物理模型) ===
        auto update_R_ = [this](const Eigen::Matrix<double, 4, 1> & measurement) {
            Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();

            // 1. 解析观测值 [y, x, z, yaw]
            double meas_y = measurement(0, 0);
            double meas_x = measurement(1, 0);
            double meas_z = measurement(2, 0);

            // 2. 计算距离信息
            // 平面距离 (用于构建旋转矩阵基向量)
            double dist_xy_sq = meas_y * meas_y + meas_x * meas_x;
            double dist_xy = std::sqrt(dist_xy_sq);
            
            // 三维欧氏距离 (用于计算由于角度抖动产生的弧长误差)
            double dist_3d = std::sqrt(dist_xy_sq + meas_z * meas_z);

            // 极近距离保护 (防止除零)
            if (dist_xy < 0.1) dist_xy = 0.1;
            if (dist_3d < 0.1) dist_3d = 0.1;

            // 3. 构建平面旋转基向量 (仅在XY平面旋转)
            // u_r: 径向单位向量 (指向目标)
            Eigen::Vector2d u_radial(meas_y / dist_xy, meas_x / dist_xy);
            // u_t: 切向单位向量 (垂直于指向，即图像水平方向)
            Eigen::Vector2d u_tangent(-meas_x / dist_xy, meas_y / dist_xy);

            // 4. === YPD 物理模型核心转换 ===
            
            // A. [Yaw/Azimuth] 方位角标准差 -> 水平切向空间标准差
            // 物理含义: 相机水平方向像素抖动导致的空间位置误差
            // 公式: Arc = r * theta
            double sigma_pos_tangent = dist_3d * std_dev_azi_angle; 

            // B. [Pitch] 俯仰角标准差 -> 垂直(Z轴)空间标准差
            // 物理含义: 相机垂直方向像素抖动导致的高度测量误差
            double sigma_pos_z = dist_3d * std_dev_ele_angle;

            // C. [Distance] 深度测量标准差 -> 径向空间标准差
            // 物理含义: 视觉算法估算深度的不确定性
            // 注意: 视觉深度的误差通常随距离增加 (线性或平方)，这里采用线性模型
            double sigma_pos_radial = dist_3d * std_dev_dist_coeff;

            // D. [Target Yaw] 目标偏航角标准差 (常数)
            double sigma_yaw = std_dev_tgt_yaw;

            // 5. 构建协方差矩阵
            // 使用基向量外积构建 XY 平面的各向异性噪声
            Eigen::Matrix2d R_pos_xy = 
                std::pow(sigma_pos_radial, 2) * (u_radial * u_radial.transpose()) +
                std::pow(sigma_pos_tangent, 2) * (u_tangent * u_tangent.transpose());

            // 填入 R 矩阵
            R.block<2, 2>(0, 0) = R_pos_xy;
            R(2, 2) = std::pow(sigma_pos_z, 2); // Z轴独立处理 (近似垂直切向)
            R(3, 3) = std::pow(sigma_yaw, 2);   // 目标Yaw独立

            return R;
        };

        // === 初始状态协方差和状态向量 ===
        Eigen::Matrix<double, 11, 11> P0;
        P0 = Eigen::Matrix<double, 11, 11>::Identity();

        Eigen::Matrix<double, 11, 1> X0;
        X0.setZero();
        X0(8, 0) = 0.26;  // 初始旋转半径设为0.26米

        // === 状态加法函数 ===
        auto x_add_ = [](const Eigen::Matrix<double, 11, 1> & x1, const Eigen::Matrix<double, 11, 1> & x2) {
            Eigen::Matrix<double, 11, 1> res;
            res = x1 + x2;
            res(6, 0) = mathutils::limit_rad(res(6, 0));  // 归一化 Yaw 角到 [-PI, PI]
            return res;
        };

        // === 状态减法函数（关键：处理 Yaw 角周期性）===
        // 计算 delta = x1 - x2，特别处理 Yaw 角 (索引 6) 的周期性 [-PI, PI]
        // 这对 IESEKF 至关重要，防止在角度接近 PI/-PI 交界处反复震荡
        auto x_minus_ = [](const Eigen::Matrix<double, 11, 1> & x1, const Eigen::Matrix<double, 11, 1> & x2) {
            Eigen::Matrix<double, 11, 1> res = x1 - x2;
            
            // // 处理 Yaw 角 (索引 6) 的周期性归一化到 [-PI, PI]
            while(res(6, 0) > M_PI) res(6, 0) -= 2 * M_PI;
            while(res(6, 0) < -M_PI) res(6, 0) += 2 * M_PI;
            
            return res;
        };

        // === 初始化 IESEKF ===
        // 参数说明：
        // - f_, h_: 状态转移和观测函数
        // - cal_F_, cal_H_: 雅可比矩阵计算函数
        // - update_Q_, update_R_: 噪声协方差更新函数
        // - P0, X0: 初始协方差和状态
        // - x_add_, x_minus_: 流形运算函数
        // - 5: 最大迭代次数（建议 3-5 次）
        // - 1e-4: 收敛阈值（状态修正量范数）
        whole_state_ekf = IESEKF_Double_Armor<4, 11>(
            f_, h_, cal_F_, cal_H_, update_Q_, update_R_, 
            P0, X0, x_add_, x_minus_,
            5,      // max_iter: 最大迭代次数
            1e-4    // stop_threshold: 收敛阈值
        );
    }

    /**
     * @brief 初始化参数调整界面
     * @details 创建OpenCV滑动条窗口，用于实时调整滤波器参数
     */
    void Tracker::parameter_adjustor_init()
    {
        // 为参数调整创建窗口
        cv::namedWindow("predictor trackbar", cv::WINDOW_AUTOSIZE);

        // 过程噪声参数调整滑动条
        cv::createTrackbar("p_coord_mant", "predictor trackbar", &p_coord_mant, 99, 0);
        cv::createTrackbar("p_coord_exp", "predictor trackbar", &p_coord_exp, 20, 0);
        cv::createTrackbar("p_yaw_mant", "predictor trackbar", &p_yaw_mant, 99, 0);
        cv::createTrackbar("p_yaw_exp", "predictor trackbar", &p_yaw_exp, 20, 0);
        cv::createTrackbar("p_r_mant", "predictor trackbar", &p_r_mant, 99, 0);
        cv::createTrackbar("p_r_exp", "predictor trackbar", &p_r_exp, 20, 0);

        // === 观测噪声参数滑动条 (物理单位：角度-度、距离-%) ===
        cv::createTrackbar("Azimuth_Deg_Int", "predictor trackbar", &azi_angle_deg_int, 50, 0);
        cv::createTrackbar("Azimuth_Deg_Frac", "predictor trackbar", &azi_angle_deg_frac, 9, 0);

        cv::createTrackbar("Pitch_Deg_Int", "predictor trackbar", &ele_angle_deg_int, 50, 0);
        cv::createTrackbar("Pitch_Deg_Frac", "predictor trackbar", &ele_angle_deg_frac, 9, 0);

        cv::createTrackbar("Distance_Coeff_%", "predictor trackbar", &dist_coeff_percent, 100, 0);

        cv::createTrackbar("TargetYaw_Deg_Int", "predictor trackbar", &tgt_yaw_deg_int, 50, 0);
        cv::createTrackbar("TargetYaw_Deg_Frac", "predictor trackbar", &tgt_yaw_deg_frac, 9, 0);

        // 装甲板模型KF参数调整滑动条
        // cv::createTrackbar("kf_yaw_mant", "predictor trackbar", &kf_yaw_mant, 99, 0);
        // cv::createTrackbar("kf_yaw_exp", "predictor trackbar", &kf_yaw_exp, 20, 0);
        // cv::createTrackbar("kf_y_mant", "predictor trackbar", &kf_y_mant, 99, 0);
        // cv::createTrackbar("kf_y_exp", "predictor trackbar", &kf_y_exp, 20, 0);
        // cv::createTrackbar("kf_x_mant", "predictor trackbar", &kf_x_mant, 99, 0);
        // cv::createTrackbar("kf_x_exp", "predictor trackbar", &kf_x_exp, 20, 0);
        // cv::createTrackbar("kf_z_mant", "predictor trackbar", &kf_z_mant, 99, 0);
        // cv::createTrackbar("kf_z_exp", "predictor trackbar", &kf_z_exp, 20, 0);
    }

    /**
     * @brief 更新滤波器参数
     * @details 从滑动条读取参数值并转换为实际的噪声协方差值
     */
    void Tracker::update_parameter()
    {
        // 更新观测噪声 (将物理单位转换为内部计算单位)
        // 角度: 度数 -> 弧度
        std_dev_azi_angle = (azi_angle_deg_int + azi_angle_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);
        std_dev_ele_angle = (ele_angle_deg_int + ele_angle_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);
        std_dev_tgt_yaw = (tgt_yaw_deg_int + tgt_yaw_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);
        
        // 距离系数: 百分比 -> 小数
        std_dev_dist_coeff = dist_coeff_percent / 100.0;
        
        // 更新过程噪声
        p_yaw = sci_to_float(p_yaw_mant, p_yaw_exp - 10);
        p_coord = sci_to_float(p_coord_mant, p_coord_exp - 10);
        p_r = sci_to_float(p_r_mant, p_r_exp - 10);
        
        // 更新装甲板KF参数
        q_kf_yaw = sci_to_float(kf_yaw_mant, kf_yaw_exp - 10);
        q_kf_y = sci_to_float(kf_y_mant, kf_y_exp - 10);
        q_kf_x = sci_to_float(kf_x_mant, kf_x_exp - 10);
        q_kf_z = sci_to_float(kf_z_mant, kf_z_exp - 10);
    }

    /**
     * @brief 更新跟踪器状态机
     * @param same_id_armor_count 检测到的同ID装甲板数量
     * @details 管理IDLE、DETECTING、TRACKING、TEMP_LOST四个状态之间的转换
     */
    void Tracker::update_tracker_state(const double same_id_armor_count)
    {
        // 更新跟踪器状态
        if (target.predictor_state == TrackingState::DETECTING) {
            if (same_id_armor_count) {
                detecting_counter++;
                if (detecting_counter > detecting_counter_threshold) {
                    // 连续检测足够次数，进入跟踪状态
                    detecting_counter = 0;
                    target.predictor_state = TrackingState::TRACKING;
                }
            }
            else {
                // 检测失败，返回空闲状态
                detecting_counter = 0;
                target.predictor_state = TrackingState::IDLE;
            }
        }
        else if (target.predictor_state == TrackingState::TRACKING) {
            if (!same_id_armor_count) {
                // 跟踪丢失，进入暂时丢失状态
                temp_lost_counter++;
                target.predictor_state = TrackingState::TEMP_LOST;
            }
        }
        else if (target.predictor_state == TrackingState::TEMP_LOST) {
            if (!same_id_armor_count) {
                temp_lost_counter++;
                if (temp_lost_counter > temp_lost_counter_threshold) {
                    // 丢失时间过长，返回空闲状态
                    temp_lost_counter = 0;
                    target.predictor_state = TrackingState::IDLE;
                }
            }
            else {
                // 重新找到目标，回到跟踪状态
                temp_lost_counter = 0;
                target.predictor_state = TrackingState::TRACKING;
            }
        }
    }

    /**
     * @brief 跟踪模型选择策略
     * @details 根据目标旋转速度在装甲板模型、整车模型和混合模型之间自动切换
     *          装甲板模型在低旋转速度下效果好，
     *          整车模型在高旋转速度下效果好，但在低速度时可能发散
     *          （该模型的Q参数对仅平移运动不敏感）
     */
    void Tracker::tracker_model_select()
    {
        target.updating_model_type = UpdatingModelType::VEHICLE_MODEL;
        return;

        // 根据旋转速度更新模型
        if (target.updating_model_type == UpdatingModelType::ARMOR_MODEL) {
            if (abs(target.yaw_state(1, 0)) > armor_model_threshold) {
                // 速度超过装甲板模型阈值，切换到混合模型
                target.updating_model_type = UpdatingModelType::BOTH;
            }
        }
        else if (target.updating_model_type == UpdatingModelType::BOTH) {
            if (abs(target.yaw_state(1, 0)) > vehicle_model_threshold) {
                // 速度超过整车模型阈值，切换到纯整车模型
                target.updating_model_type = UpdatingModelType::VEHICLE_MODEL;
            }
            else if (abs(target.yaw_state(1, 0)) < armor_model_threshold) {
                // 速度低于装甲板模型阈值，切换到纯装甲板模型
                target.updating_model_type = UpdatingModelType::ARMOR_MODEL;
            }
        }
        else if (target.updating_model_type == UpdatingModelType::VEHICLE_MODEL) {
            if (abs(target.yaw_state(1, 0)) < vehicle_model_threshold) {
                // 速度低于整车模型阈值，切换到混合模型
                target.updating_model_type = UpdatingModelType::BOTH;
            }
        }
    }

    /**
     * @brief 重置偏航角卡尔曼滤波器
     * @details 用当前观测值重新初始化偏航角滤波器状态
     */
    void Tracker::reset_yaw_kf()
    {
        target.yaw_measurement << target.tracked_measurement(3, 0);
        target.yaw_state(0, 0) = target.tracked_measurement(3, 0);
        yaw_kf.reset(target.yaw_state); 
    }

    /**
     * @brief 重置X坐标卡尔曼滤波器
     */
    void Tracker::reset_armor_x_kf()
    {
        target.armor_x_measurement << target.tracked_measurement(1, 0);
        target.armor_x_state << target.tracked_measurement(1, 0), 0;
        armor_x_kf.reset(target.armor_x_state); 
    }

    /**
     * @brief 重置Y坐标卡尔曼滤波器
     */
    void Tracker::reset_armor_y_kf()
    {
        target.armor_y_measurement << target.tracked_measurement(0, 0);
        target.armor_y_state << target.tracked_measurement(0, 0), 0;
        armor_y_kf.reset(target.armor_y_state); 
    }

    /**
     * @brief 重置Z坐标卡尔曼滤波器
     */
    void Tracker::reset_armor_z_kf()
    {
        target.armor_z_measurement << target.tracked_measurement(2, 0);
        target.armor_z_state << target.tracked_measurement(2, 0), 0;
        armor_z_kf.reset(target.armor_z_state); 
    }

    /**
     * @brief 重置整车状态扩展卡尔曼滤波器
     * @details 从当前观测值推断车辆中心位置并重新初始化EKF
     */
    void Tracker::reset_whole_state_ekf()
    {
        // 从装甲板位置和朝向推算车辆中心位置
        double yc = target.tracked_measurement(0, 0) + 0.26 * cos(target.tracked_measurement(3, 0));
        double xc = target.tracked_measurement(1, 0) + 0.26 * sin(target.tracked_measurement(3, 0));
        
        // 设置初始状态：[yc, 0, xc, 0, zc, 0, yaw, 0, 0.26, 0, 0]
        target.tracked_state << yc, 0, xc, 0, target.tracked_measurement(2, 0), 0, 
                                target.tracked_measurement(3, 0), 0, 0.26, 0, 0;

        whole_state_ekf.reset(target.tracked_state);

        // 重置相关参数
        target.another_r = 0.26;
        target.dz = 0;
        target.ab_counter = 0;
        yaw_speed_diverge_counter = 0;
        rotate_counter = 0;
        target.vehicle_model_trust = false;
    }

    /**
     * @brief 限制旋转半径在合理范围内
     * @details 将估计的旋转半径限制在合理范围内
     */
    void Tracker::radium_limit()
    {
        // 限制半径范围
        if (target.tracked_state(8, 0) < 0.12) {
            target.tracked_state(8, 0) = 0.12;
        }
        else if (target.tracked_state(8, 0) > 0.4) {
            target.tracked_state(8, 0) = 0.4;
        }

        if (target.tracked_state(8, 0) + target.tracked_state(9, 0) < 0.12) {
            target.tracked_state(9, 0) = 0.12 - target.tracked_state(8, 0);
        }
        else if (target.tracked_state(8, 0) + target.tracked_state(9, 0) > 0.4) {
            target.tracked_state(9, 0) = 0.4 - target.tracked_state(8, 0);
        }

        whole_state_ekf.reset(target.tracked_state);
    }

    /**
     * @brief 检查扩展卡尔曼滤波器是否发散
     * @param attitude_yaw 机器人当前姿态偏航角
     * @return true表示检测到发散，需要重置滤波器
     * @details 根据估计的偏航角和偏航角速度检查整车模型是否发散
     */
    bool Tracker::check_ekf_divergence(const double attitude_yaw)
    {
        // 检查偏航角是否超出相机视野范围
        bool yaw_diverge = false;
        if (target.tracked_state(6, 0) > M_PI_2 - attitude_yaw || 
            target.tracked_state(6, 0) < -M_PI_2 - attitude_yaw) {
            yaw_diverge = true;
        }

        // 检查两种模型对偏航角速度估计的一致性
        if ((target.tracked_state(7, 0) > 0 && target.yaw_state(1, 0) < 0) || 
            (target.tracked_state(7, 0) < 0 && target.yaw_state(1, 0) > 0)) {
            yaw_speed_diverge_counter++;
        }
        else {
            yaw_speed_diverge_counter = 0;
        }

        bool yaw_speed_diverge = false;
        if (yaw_speed_diverge_counter > yaw_speed_diverge_threshold) {
            yaw_speed_diverge_counter = 0;
            yaw_speed_diverge = true;
        }

        return yaw_diverge || yaw_speed_diverge;
    }

} // namespace predict

