/**
 * @file Kalman.hpp
 * @brief 卡尔曼滤波器模块 - 为目标跟踪提供线性滤波算法
 * @author Cao Jingyan
 * @date 2025/11/21
 * 
 * 该模块提供：
 * 1. 通用的线性卡尔曼滤波器实现
 * 2. 支持自定义状态空间和观测空间维度
 * 3. 线性系统的状态预测和更新
 * 4. 灵活的函数接口设计，支持时变系统矩阵
 */

#ifndef _PREDICT_KALMAN_HPP_
#define _PREDICT_KALMAN_HPP_

#include <Eigen/Dense>
#include <functional>

namespace mathutils
{
    /**
     * @class Kalman
     * @brief 线性卡尔曼滤波器模板类
     * @tparam V_Z 观测向量维度，默认为1（单变量观测）
     * @tparam V_X 状态向量维度，默认为2（位置+速度模型）
     * @details 实现标准的线性卡尔曼滤波算法，适用于线性系统
     *          支持时变的系统矩阵和噪声协方差矩阵
     *          通过函数指针实现动态矩阵更新
     */
    template <int V_Z = 1, int V_X = 2>
    class Kalman
    {
    public:
        // === 类型定义 ===
        /// @brief 观测协方差矩阵类型 [V_Z × V_Z]
        using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
        
        /// @brief 状态协方差矩阵类型 [V_X × V_X]
        using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
        
        /// @brief 观测矩阵类型 [V_Z × V_X]
        using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
        
        /// @brief 卡尔曼增益矩阵类型 [V_X × V_Z]
        using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
        
        /// @brief 状态向量类型 [V_X × 1]
        using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
        
        /// @brief 观测向量类型 [V_Z × 1]
        using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;

        // === 函数指针类型定义 ===
        /// @brief 观测矩阵更新函数类型：H = f(z)
        using Func_zxd_z1d = std::function<Matrix_zxd(const Matrix_z1d &)>;
        
        /// @brief 状态转移矩阵更新函数类型：A = f(x)；过程噪声协方差更新函数类型：Q = f(x)
        using Func_xxd_x1d = std::function<Matrix_xxd(const Matrix_x1d &)>;
        
        /// @brief 观测噪声协方差更新函数类型：R = f(z)
        using Func_zzd_z1d = std::function<Matrix_zzd(const Matrix_z1d &)>;
        

    private:
        // === 滤波器系统矩阵更新函数 ===
        /// @brief 状态转移矩阵更新函数
        Func_xxd_x1d update_A;

        /// @brief 状态转移矩阵 A
        Matrix_xxd A;
        
        /// @brief 观测矩阵更新函数
        Func_zxd_z1d update_H;

        /// @brief 观测矩阵 H
        Matrix_zxd H;

        // === 噪声协方差矩阵 ===
        /// @brief 过程噪声协方差更新函数
        Func_xxd_x1d update_Q;
        
        /// @brief 过程噪声协方差矩阵 Q
        Matrix_xxd Q;
        
        /// @brief 观测噪声协方差更新函数
        Func_zzd_z1d update_R;
        
        /// @brief 观测噪声协方差矩阵 R
        Matrix_zzd R;

        // === 状态协方差矩阵 ===
        /// @brief 先验状态协方差矩阵 P⁻
        Matrix_xxd P_pri;
        
        /// @brief 后验状态协方差矩阵 P⁺
        Matrix_xxd P_post;

        /// @brief 卡尔曼增益矩阵 K
        Matrix_xzd K;

        // === 状态向量 ===
        /// @brief 先验状态估计 x⁻
        Matrix_x1d X_pri;
        
        /// @brief 后验状态估计 x⁺
        Matrix_x1d X_post;

    public:
        /**
         * @brief 默认构造函数
         */
        Kalman() = default;
        
        /**
         * @brief 带参数构造函数 - 初始化卡尔曼滤波器的所有参数
         * @param update_A_ 状态转移矩阵更新函数 A = f(x)
         * @param update_H_ 观测矩阵更新函数 H = f(z)
         * @param update_Q_ 过程噪声协方差更新函数 Q = f(x)
         * @param update_R_ 观测噪声协方差更新函数 R = f(z)
         * @param P0 初始状态协方差矩阵
         * @param X0 初始状态向量
         * @details 完整初始化滤波器，设置所有必要的函数和初始条件
         *          支持时变系统，各矩阵可根据当前状态或观测动态调整
         */
        Kalman(const Func_xxd_x1d &update_A_, const Func_zxd_z1d &update_H_,
                const Func_xxd_x1d &update_Q_, const Func_zzd_z1d &update_R_,
                const Matrix_xxd &P0, const Matrix_x1d &X0)
        : update_A(update_A_), update_H(update_H_), update_Q(update_Q_), update_R(update_R_), 
            P_post(P0), X_post(X0)
        {
        }

        /**
         * @brief 重置滤波器状态
         * @param X0 新的初始状态向量
         * @details 仅重置状态向量，保持其他参数和协方差矩阵不变
         */
        void reset(const Matrix_x1d &X0)
        {
            X_post = X0;
        }

        /**
         * @brief 卡尔曼滤波更新步骤 - 预测和校正的完整过程
         * @param Z 当前时刻的观测向量
         * @return Matrix_x1d 后验状态估计 x⁺(k+1|k+1)
         * @details 执行完整的卡尔曼滤波周期：
         *          1. 更新系统矩阵：A, H, Q, R
         *          2. 预测步骤：x⁻ = A·x⁺, P⁻ = A·P⁺·Aᵀ + Q
         *          3. 校正步骤：K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
         *                      x⁺ = x⁻ + K·(z - H·x⁻)
         *                      P⁺ = (I - K·H)·P⁻
         */
        Matrix_x1d update(const Matrix_z1d &Z)
        {
            // 更新过程噪声协方差矩阵
            Q = update_Q(X_post);

            // 更新观测噪声协方差矩阵
            R = update_R(Z);

            // 更新状态转移矩阵
            A = update_A(X_post);

            // 更新观测矩阵
            H = update_H(Z);

            // === 预测步骤 ===
            // 预测下一时刻的状态
            X_pri = A * X_post;

            // 预测状态协方差
            P_pri = A * P_post * A.transpose() + Q; 

            // === 校正步骤 ===
            // 计算卡尔曼增益
            K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse(); 

            // 状态校正（融合观测信息）
            X_post = X_pri + K * (Z - H * X_pri);

            // 更新后验状态协方差
            P_post = (Matrix_xxd::Identity() - K * H) * P_pri;

            return X_post;
        }
    }; // class Kalman

} // namespace predict

#endif /* _PREDICT_KALMAN_HPP_ */
