/**
 * @file Tracker.hpp
 * @brief 目标跟踪器模块 - 实现多模型目标跟踪和状态估计
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 该模块提供：
 * 1. 多模型自适应跟踪算法
 * 2. 扩展卡尔曼滤波和线性卡尔曼滤波的组合使用
 * 3. 基于旋转速度的模型自动切换
 * 4. 跟踪状态管理和异常检测
 */

#ifndef _PREDICT_TRACKER_HPP_
#define _PREDICT_TRACKER_HPP_

// modules
#include "common.hpp"
#include "IESEKF.hpp"
#include "Kalman.hpp"
#include "math_tools.hpp"
#include "IESEKF_Double_Armor.hpp"

// packages
#include <iostream>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <cfloat>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace mathutils;

namespace predict
{
    using namespace std::chrono;

    /**
     * @class Tracker
     * @brief 目标跟踪器类 - 多模型自适应跟踪系统
     * @details 实现了装甲板模型和整车模型的组合跟踪：
     *          - 装甲板模型：使用线性KF跟踪装甲板的x,y,z坐标和yaw角
     *          - 整车模型：使用EKF跟踪车辆中心和旋转参数
     *          - 根据目标旋转速度自动选择合适的跟踪模型
     *          - 具有完整的状态机管理和异常检测机制
     */
    class Tracker
    {
    public:
        /// @brief 时间点类型别名
        using TP = std::chrono::high_resolution_clock::time_point;

        /// @brief 二维卡尔曼滤波器类型别名（观测维度1，状态维度2）
        using filter_2d = Kalman<1, 2>;
        using Matx1_2d = filter_2d::Matrix_x1d;
        using Matxx_2d = filter_2d::Matrix_xxd;
        using Matxz_2d = filter_2d::Matrix_xzd;
        using Matz1_2d = filter_2d::Matrix_z1d;
        using Matzx_2d = filter_2d::Matrix_zxd;
        using Matzz_2d = filter_2d::Matrix_zzd;

        // === 跟踪参数常量 ===
        /// @brief 相机视野内最大可接受偏航角 (弧度)
        const double max_yaw_accept = 0.85;

        /// @brief 同一位置判断阈值 (米)，用于判断装甲板是否发生跳变
        const double same_position_threshold = 0.2;

        /// @brief 检测阶段计数阈值，连续检测超过此值进入跟踪状态
        const int detecting_counter_threshold = 4;
        
        // on axcl
        /// @brief 暂时丢失计数阈值，丢失超过此值返回空闲状态
        const int temp_lost_counter_threshold = 30;

        const double yaw_speed_diverge_threshold = 3;

        /// @brief 偏航角速度发散检测阈值
        const int yaw_speed_diverge_counter_threshold = 30;

        /// @brief 装甲板模型适用阈值 (弧度/秒)，低于此速度使用装甲板模型
        const double armor_model_threshold = 0.7;
        
        /// @brief 整车模型适用阈值 (弧度/秒)，高于此速度使用整车模型
        const double vehicle_model_threshold = 2.7;

        /// @brief 最少旋转计数阈值，低于此值不信任整车模型
        const int least_rotate_count = 3;

        const double outpost_r = 0.33;
        const double outpost_yaw_speed = 3.14;
        const double outpost_fix_yaw_speed_threshold = 2.5; 

    private:
        // === 配置参数 ===
        /// @brief 调试模式标志
        bool debug = false;
        
        /// @brief 参数调整模式标志
        bool adjust = false;

        // === 滤波器参数（科学计数法表示，支持实时调整） ===
        /// @brief 坐标过程噪声尾数
        int p_coord_mant = 1;
        /// @brief 坐标过程噪声指数
        int p_coord_exp = 2 + 10;
        /// @brief 偏航角过程噪声尾数
        int p_yaw_mant = 4;
        /// @brief 偏航角过程噪声指数
        int p_yaw_exp = 2 + 10;
        /// @brief 旋转半径过程噪声尾数
        int p_r_mant = 1;
        /// @brief 旋转半径过程噪声指数
        int p_r_exp = -5 + 10;

        // === 观测噪声参数 (基于球坐标系 YPD + Yaw 物理模型) ===
        
        // 1. 方位角标准差 (Azimuth Angle Std Dev) - 单位: 度
            // 物理意义: 相机水平方向像素抖动导致的角度不确定性
        // 默认: 3.6°
        int azi_angle_deg_int = 36;  // 0.1度 * 36 = 3.6°
        int azi_angle_deg_frac = 0;  // 小数部分 (0.0)

        // 2. 俯仰角标准差 (Elevation/Pitch Angle Std Dev) - 单位: 度
        // 物理意义: 相机垂直方向像素抖动导致的角度不确定性
        // 经验值: 0.5° ~ 2.0° (云台Pitch轴震动可能较大)
        // 默认: 3.6°
        int ele_angle_deg_int = 36;  // 0.1度 * 36 = 3.6°
        int ele_angle_deg_frac = 0;  // 小数部分 (0.0)

        // 3. 距离误差系数 (Distance Error Coefficient) - 无量纲
        // 物理意义: 距离每增加1米，深度测量误差增加多少米 (误差 = 距离 * 系数)
        // 默认: 0.20 (20%)
        int dist_coeff_percent = 20;  // 百分比表示 (20% = 0.20)

        // 4. 目标偏航角标准差 (Target Yaw Std Dev) - 单位: 度
        // 物理意义: 神经网络解算装甲板朝向的角度不确定性
        // 经验值: 5° ~ 15°
        // 默认: 10°
        int tgt_yaw_deg_int = 200;     // 0.1度 * 200 = 20°
        int tgt_yaw_deg_frac = 0;     // 小数部分 (0.0)

        // 观测噪声参数
        int r_ycoord_mant = 3;
        int r_ycoord_exp = -4 + 10;
        int r_xcoord_mant = 3;
        int r_xcoord_exp = -4 + 10;
        int r_zcoord_mant = 3;
        int r_zcoord_exp = -4 + 10;
        int r_yaw_mant = 3;
        int r_yaw_exp = -3 + 10;
        
        // 装甲板KF噪声参数
        int kf_yaw_mant = 15;
        int kf_yaw_exp = -1 + 10;
        int kf_y_mant = 5;
        int kf_y_exp = 0 + 10;
        int kf_x_mant = 5;
        int kf_x_exp = 0 + 10;
        int kf_z_mant = 5;
        int kf_z_exp = 0 + 10;

        // === 计算后的实际物理参数 ===
        double std_dev_azi_angle = (azi_angle_deg_int + azi_angle_deg_frac / 10.0) * 0.1 * (M_PI / 180.0); // 方位角标准差 (弧度)
        double std_dev_ele_angle = (ele_angle_deg_int + ele_angle_deg_frac / 10.0) * 0.1 * (M_PI / 180.0); // 俯仰角标准差 (弧度)
        double std_dev_dist_coeff = dist_coeff_percent / 100.0;                                           // 距离误差系数 (无量纲)
        double std_dev_tgt_yaw = (tgt_yaw_deg_int + tgt_yaw_deg_frac / 10.0) * 0.1 * (M_PI / 180.0);      // 目标yaw标准差 (弧度)
        
        // 过程噪声参数
        double p_yaw = sci_to_float(p_yaw_mant, p_yaw_exp - 10);
        double p_coord = sci_to_float(p_coord_mant, p_coord_exp - 10);
        double p_r = sci_to_float(p_r_mant, p_r_exp - 10);
        
        // 装甲板KF参数
        double r_xcoord = sci_to_float(r_xcoord_mant, r_xcoord_exp - 10);
        double r_ycoord = sci_to_float(r_ycoord_mant, r_ycoord_exp - 10);
        double r_zcoord = sci_to_float(r_zcoord_mant, r_zcoord_exp - 10);
        double r_yaw = sci_to_float(r_yaw_mant, r_yaw_exp - 10);
        double q_kf_yaw = sci_to_float(kf_yaw_mant, kf_yaw_exp - 10);
        double q_kf_y = sci_to_float(kf_y_mant, kf_y_exp - 10);
        double q_kf_x = sci_to_float(kf_x_mant, kf_x_exp - 10);
        double q_kf_z = sci_to_float(kf_z_mant, kf_z_exp - 10);

        // === 状态变量 ===
        /// @brief 上次更新时间点
        TP last_tp;
        
        /// @brief 时间间隔 (秒)
        double dt;

        /// @brief 目标跟踪状态结构体
        Target target;

        /// @brief 整车状态迭代扩展卡尔曼滤波器 (观测维度4，状态维度9)
        IESEKF_Double_Armor<4, 11> whole_state_ekf;

        /// @brief 偏航角速度发散计数器
        int yaw_speed_diverge_counter;

        // === 装甲板模型卡尔曼滤波器组 ===
        /// @brief 偏航角滤波器 [yaw, yaw_velocity]
        filter_2d yaw_kf;

        /// @brief X坐标滤波器 [x, vx]
        filter_2d armor_x_kf;

        /// @brief Y坐标滤波器 [y, vy]
        filter_2d armor_y_kf;

        /// @brief Z坐标滤波器 [z, vz]
        filter_2d armor_z_kf;

        /// @brief 检测阶段计数器
        int detecting_counter;
        
        /// @brief 暂时丢失计数器
        int temp_lost_counter;

        /// @brief 暂时丢失时间点
        TP temp_lost_tp;

        /// @brief 目标旋转计数器
        int rotate_counter;

    private:
        /**
         * @brief 科学计数法转浮点数
         * @param mant 尾数
         * @param exp 指数
         * @return 转换后的浮点数值
         */
        inline float sci_to_float(int mant, int exp) {
            return mant * std::pow(10.0f, exp);
        }

        std::vector<Eigen::Vector4d> armor_xyza_list();

        /**
         * @brief 初始化目标状态
         * @details 设置目标的初始状态和参数
         */
        void target_init();
        
        /**
         * @brief 初始化装甲板模型卡尔曼滤波器
         * @details 设置四个独立的KF：yaw, x, y, z坐标滤波器
         */
        void armor_state_kf_init();
        
        /**
         * @brief 初始化整车状态扩展卡尔曼滤波器
         * @details 设置EKF的状态转移函数、观测函数和雅可比矩阵
         */
        void whole_state_ekf_init();
        
        /**
         * @brief 初始化参数调整界面
         * @details 创建OpenCV滑动条用于实时参数调整
         */
        void parameter_adjustor_init();

        /**
         * @brief 更新滤波器参数
         * @details 从滑动条读取参数值并更新滤波器配置
         */
        void update_parameter();

        // === 滤波器重置函数组 ===
        void reset_yaw_kf();        ///< 重置偏航角滤波器
        void reset_armor_x_kf();    ///< 重置X坐标滤波器
        void reset_armor_y_kf();    ///< 重置Y坐标滤波器
        void reset_armor_z_kf();    ///< 重置Z坐标滤波器
        void reset_whole_state_ekf(); ///< 重置整车状态EKF

        /**
         * @brief 更新跟踪器状态机
         * @param same_id_armor_count 检测到同ID装甲板的数量
         * @details 根据检测结果更新跟踪器在IDLE、DETECTING、TRACKING、TEMP_LOST之间的状态转换
         */
        void update_tracker_state(const double same_id_armor_count);

        /**
         * @brief 跟踪模型选择策略
         * @details 根据目标旋转速度在装甲板模型、整车模型和混合模型之间动态切换
         */
        void tracker_model_select();

        /**
         * @brief 旋转半径限制
         * @details 将估计的旋转半径限制在合理范围内
         */
        void radium_limit();

        /**
         * @brief 检查EKF发散
         * @param attitude_yaw 机器人当前偏航角
         * @return true表示检测到发散，需要重置
         * @details 通过比较两种模型对偏航角估计和偏航角速度估计的一致性检测模型发散
         */
        bool check_ekf_divergence(const double attitude_yaw);

    public:
        /**
         * @brief 带参数构造函数
         * @param debug_ 调试模式标志
         * @param adjust_ 参数调整模式标志
         */
        explicit Tracker();

        void set_debug(bool debug_);

        void set_adjust(bool adjust_);

        /**
         * @brief 获取当前跟踪状态
         * @return 当前跟踪器状态
         */
        TrackingState get_tracker_state();
        
        /**
         * @brief 获取目标跟踪信息
         * @return 目标状态结构体的常引用
         */
        const Target& get_target();

        /**
         * @brief 重置目标跟踪器
         * @param measurement 初始观测值 [y, x, z, yaw]
         * @param tp 时间戳
         * @return 重置后的目标状态
         * @details 用新的观测值重新初始化所有滤波器，进入DETECTING状态
         */
        const Target& reset_target(const Eigen::Matrix<double, 4, 1> &measurement, TP &tp);

        Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id);

        int match_armor_id(const Eigen::Matrix<double, 4, 1> &measurement);

        /**
         * @brief 执行目标跟踪更新
         * @param measurement 当前观测值 [y, x, z, yaw]
         * @param secondary_measurement 当前备选观测值 [y, x, z, yaw]
         * @param same_id_armor_count 同ID装甲板检测数量
         * @param tp 当前时间戳
         * @param attitude_yaw 机器人当前姿态偏航角
         * @return 更新后的目标状态
         * @details 执行完整的跟踪流程：预测、模型选择、状态更新、异常检测
         */
        const Target& track(const Eigen::Matrix<double, 4, 1> &measurement, const Eigen::Matrix<double, 4, 1> &secondary_measurement, const int same_id_armor_count,
                                const int tag_id, const TP &tp, const double attitude_yaw);

    }; // class Tracker

} // namespace predict

#endif // _PREDICT_TRACKER_HPP_
