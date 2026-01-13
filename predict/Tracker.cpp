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
     * @param same_id_armor_count 检测到的同ID装甲板数量
     * @param tp 当前时间戳
     * @param attitude_yaw 机器人当前姿态偏航角
     * @return 更新后的目标状态引用
     * @details 执行完整的跟踪流程：预测、更新、模型选择、异常检测
     */
    const Target& Tracker::track(const Eigen::Matrix<double, 4, 1> &measurement, const int same_id_armor_count,
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
        target.tracked_measurement = measurement;

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
                if (min_position_diff < same_position_threshold) {
                    target.tracked_state = whole_state_ekf.update(target.tracked_measurement);
                    if (debug)
                        cout << "[predict] ekf update" << std::endl;
                }
                else {
                    // 装甲板跳变处理
                    // 记录高度差和不同装甲板对的半径差异
                    target.dz = target.tracked_state(4, 0) - target.tracked_measurement(2, 0);
                    target.tracked_state(6, 0) = target.tracked_measurement(3, 0);  // 更新偏航角
                    target.tracked_state(4, 0) = target.tracked_measurement(2, 0);  // 更新Z坐标
                    std::swap(target.tracked_state(8, 0), target.another_r);        // 交换半径

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

                    // 重置EKF状态
                    whole_state_ekf.reset(target.tracked_state);

                    if (debug)
                        cout << "[predict] armor jump" << std::endl;
                }

                // 限制旋转半径在合理范围内
                radium_limit();

                // 检查EKF是否发散
                if (check_ekf_divergence(attitude_yaw)) {
                    reset_whole_state_ekf();

                    if (debug)
                        cout << "[predict] vehicle model converge" << endl;
                }
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

            R << r_yaw;

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

            R << r_ycoord;

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
            R << r_xcoord;  // X坐标观测噪声方差
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
            R << r_zcoord;  // Z坐标观测噪声方差
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
        // 状态: y, vy, x, vx, z, vz, yaw(-∞, +∞), vyaw, r
        // 观测: y, x, z, yaw(-∞, +∞)
        auto f_ = [this](const Eigen::Matrix<double, 9, 1> & x) {
            Eigen::Matrix<double, 9, 1> x_pri = x;
            // 积分更新位置和角度
            x_pri(0, 0) += x(1, 0) * dt;  // y = y + vy * dt
            x_pri(2, 0) += x(3, 0) * dt;  // x = x + vx * dt
            x_pri(4, 0) += x(5, 0) * dt;  // z = z + vz * dt
            x_pri(6, 0) += x(7, 0) * dt;  // yaw = yaw + vyaw * dt
            return x_pri;
        };

        // === 状态转移雅可比矩阵 ===
        auto cal_F_ = [this](const Eigen::Matrix<double, 9, 1> & x) {
            Eigen::Matrix<double, 9, 9> F;
            F << 1,   dt, 0,   0,   0,   0,   0,   0,   0,
                 0,   1,   0,   0,   0,   0,   0,   0,   0,
                 0,   0,   1,   dt, 0,   0,   0,   0,   0, 
                 0,   0,   0,   1,   0,   0,   0,   0,   0,
                 0,   0,   0,   0,   1,   dt, 0,   0,   0,
                 0,   0,   0,   0,   0,   1,   0,   0,   0,
                 0,   0,   0,   0,   0,   0,   1,   dt, 0,
                 0,   0,   0,   0,   0,   0,   0,   1,   0,
                 0,   0,   0,   0,   0,   0,   0,   0,   1;
            return F;
        };

        // === 观测函数 ===
        // 从车辆中心状态计算装甲板位置
        auto h_ = [](const Eigen::Matrix<double, 9, 1> & x) {
            Eigen::Matrix<double, 4, 1> z;
            z(0, 0) = x(0, 0) - x(8, 0) * cos(x(6, 0));  // ya = yc - r * cos(yaw)
            z(1, 0) = x(2, 0) - x(8, 0) * sin(x(6, 0));  // xa = xc - r * sin(yaw)
            z(2, 0) = x(4, 0);                            // za = zc
            z(3, 0) = x(6, 0);                            // yaw_a = yaw_c
            return z;
        };

        // === 观测雅可比矩阵 ===
        auto cal_H_ = [](const Eigen::Matrix<double, 9, 1> & x) {
            Eigen::Matrix<double, 4, 9> H;
            H << 1,   0,   0,   0,   0,   0,   x(8, 0)*sin(x(6, 0)), 0,   -cos(x(6, 0)),
                 0,   0,   1,   0,   0,   0,   -x(8, 0)*cos(x(6, 0)),0,   -sin(x(6, 0)),
                 0,   0,   0,   0,   1,   0,   0,                    0,   0,
                 0,   0,   0,   0,   0,   0,   1,                    0,   0;
            return H;
        };

        // === 过程噪声协方差矩阵 ===
        auto update_Q_ = [this](const Eigen::Matrix<double, 9, 1> & x) {
            Eigen::Matrix<double, 9, 9> Q;
            // 连续白噪声加速度模型
            double q_x_x = pow(dt, 4) / 4 * p_coord, q_x_vx = pow(dt, 3) / 2 * p_coord, q_vx_vx = pow(dt, 2) * p_coord;
            double q_y_y = pow(dt, 4) / 4 * p_yaw, q_y_vy = pow(dt, 3) / 2 * p_yaw, q_vy_vy = pow(dt, 2) * p_yaw;
            double q_r = pow(dt, 4) / 4 * p_r;

            Q << q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
                 q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
                 0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
                 0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
                 0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
                 0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
                 0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
                 0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
                 0,      0,      0,      0,      0,      0,      0,      0,      q_r;
            return Q;
        };

        // TODO: 噪声从ypd投影到xyz
        // === 观测噪声协方差矩阵 ===
        auto update_R_ = [this](const Eigen::Matrix<double, 4, 1> & x) {
            Eigen::Matrix<double, 4, 4> R;
            // 观测噪声与观测值成正比（距离越远噪声越大）
            R << abs(r_ycoord * x(0, 0)), 0,       0,       0,
                 0,       abs(r_xcoord * x(1, 0)), 0,       0,
                 0,       0,       abs(r_zcoord * x(2, 0)), 0,
                 0,       0,       0,       r_yaw;
            return R;
        };

        // === 初始状态协方差和状态向量 ===
        Eigen::Matrix<double, 9, 9> P0;
        P0 = Eigen::Matrix<double, 9, 9>::Identity();

        Eigen::Matrix<double, 9, 1> X0;
        X0.setZero();
        X0(8, 0) = 0.26;  // 初始旋转半径设为0.26米

        // === 状态加法函数 ===
        auto x_add_ = [this](const Eigen::Matrix<double, 9, 1> & x1, const Eigen::Matrix<double, 9, 1> & x2) {
            Eigen::Matrix<double, 9, 1> res;
            res = x1 + x2;
            return res;
        };

        // 初始化EKF
        whole_state_ekf.reset(f_, h_, cal_F_, cal_H_, update_Q_, update_R_, P0, X0, x_add_);
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

        // 观测噪声参数滑动条
        cv::createTrackbar("r_xcoord_mant", "predictor trackbar", &r_xcoord_mant, 99, 0);
        cv::createTrackbar("r_xcoord_exp", "predictor trackbar", &r_xcoord_exp, 20, 0);
        cv::createTrackbar("r_ycoord_mant", "predictor trackbar", &r_ycoord_mant, 99, 0);
        cv::createTrackbar("r_ycoord_exp", "predictor trackbar", &r_ycoord_exp, 20, 0);
        cv::createTrackbar("r_zcoord_mant", "predictor trackbar", &r_zcoord_mant, 99, 0);
        cv::createTrackbar("r_zcoord_exp", "predictor trackbar", &r_zcoord_exp, 20, 0);
        cv::createTrackbar("r_yaw_mant", "predictor trackbar", &r_yaw_mant, 99, 0);
        cv::createTrackbar("r_yaw_exp", "predictor trackbar", &r_yaw_exp, 20, 0);

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
        r_xcoord = sci_to_float(r_xcoord_mant, r_xcoord_exp - 10);
        r_ycoord = sci_to_float(r_ycoord_mant, r_ycoord_exp - 10);
        r_zcoord = sci_to_float(r_zcoord_mant, r_zcoord_exp - 10);
        r_yaw = sci_to_float(r_yaw_mant, r_yaw_exp - 10);
        p_yaw = sci_to_float(p_yaw_mant, p_yaw_exp - 10);
        p_coord = sci_to_float(p_coord_mant, p_coord_exp - 10);
        p_r = sci_to_float(p_r_mant, p_r_exp - 10);
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
        
        // 设置初始状态：[yc, 0, xc, 0, zc, 0, yaw, 0, 0.26]
        target.tracked_state << yc, 0, xc, 0, target.tracked_measurement(2, 0), 0, 
                                target.tracked_measurement(3, 0), 0, 0.26;

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
            whole_state_ekf.reset(target.tracked_state);
        }
        else if (target.tracked_state(8, 0) > 0.4) {
            target.tracked_state(8, 0) = 0.4;
            whole_state_ekf.reset(target.tracked_state);
        }
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

