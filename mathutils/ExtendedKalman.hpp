/**
 * @file ExtendedKalman.hpp
 * @brief 扩展卡尔曼滤波器模块 - 为目标跟踪和状态估计提供滤波算法
 * @author Cao Jingyan
 * @date 2025/11/16
 * 
 * 该模块提供：
 * 1. 通用的扩展卡尔曼滤波器实现
 * 2. 支持自定义状态空间和观测空间维度
 * 3. 非线性系统的状态预测和更新
 * 4. 灵活的函数接口设计，支持复杂运动模型
 */

#ifndef _PREDICT_EXTENDED_KALMAN_HPP_
#define _PREDICT_EXTENDED_KALMAN_HPP_

#include <Eigen/Dense>
#include <functional>

namespace mathutils
{
    /**
     * @class ExtendedKalman
     * @brief 扩展卡尔曼滤波器模板类
     * @tparam V_Z 观测向量维度，默认为4（通常包含位置和角度观测）
     * @tparam V_X 状态向量维度，默认为9（通常包含位置、速度、加速度）
     * @details 实现标准的EKF算法，包括预测步骤和更新步骤
     *          支持非线性状态转移函数和非线性观测函数
     *          通过函数指针实现灵活的模型定义
     */
    template <int V_Z = 4, int V_X = 9>
    class ExtendedKalman
    {
    public:
        // === 类型定义 ===
        /// @brief 观测协方差矩阵类型 [V_Z × V_Z]
        using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
        
        /// @brief 状态协方差矩阵类型 [V_X × V_X]
        using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
        
        /// @brief 观测雅可比矩阵类型 [V_Z × V_X]
        using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
        
        /// @brief 卡尔曼增益矩阵类型 [V_X × V_Z]
        using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
        
        /// @brief 状态向量类型 [V_X × 1]
        using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
        
        /// @brief 观测向量类型 [V_Z × 1]
        using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;

        // === 函数指针类型定义 ===
        /// @brief 状态转移函数类型：x(k+1) = f(x(k))
        using Func_x1d_x1d = std::function<Matrix_x1d(const Matrix_x1d &)>;
        
        /// @brief 观测函数类型：z(k) = h(x(k))
        using Func_z1d_x1d = std::function<Matrix_z1d(const Matrix_x1d &)>;
        
        /// @brief 观测雅可比函数类型：H = ∂h/∂x
        using Func_zxd_x1d = std::function<Matrix_zxd(const Matrix_x1d &)>;
        
        /// @brief 状态雅可比函数类型：F = ∂f/∂x；过程噪声协方差更新函数类型
        using Func_xxd_x1d = std::function<Matrix_xxd(const Matrix_x1d &)>;
        
        /// @brief 观测噪声协方差更新函数类型
        using Func_zzd_z1d = std::function<Matrix_zzd(const Matrix_z1d &)>;
        
        /// @brief 状态加法函数类型
        using Add_func = std::function<Matrix_x1d(const Matrix_x1d &, const Matrix_x1d &)>;

    private:
        // === 滤波器核心函数 ===
        /// @brief 状态转移函数 f(x)
        Func_x1d_x1d f;
        
        /// @brief 观测函数 h(x)
        Func_z1d_x1d h;
        
        /// @brief 状态雅可比计算函数
        Func_xxd_x1d calculate_F;
        
        /// @brief 状态雅可比矩阵 F = ∂f/∂x
        Matrix_xxd F;
        
        /// @brief 观测雅可比计算函数
        Func_zxd_x1d calculate_H;
        
        /// @brief 观测雅可比矩阵 H = ∂h/∂x
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

        /// @brief 状态加法函数
        Add_func x_add;

    public:
        /**
         * @brief 默认构造函数
         */
        ExtendedKalman() = default;

        /**
         * @brief 带参数构造函数 - 初始化滤波器的所有参数
         * @param f_ 状态转移函数 x(k+1) = f(x(k))
         * @param h_ 观测函数 z(k) = h(x(k))
         * @param cal_F_ 状态雅可比计算函数 F = ∂f/∂x
         * @param cal_H_ 观测雅可比计算函数 H = ∂h/∂x
         * @param update_Q_ 过程噪声协方差更新函数
         * @param update_R_ 观测噪声协方差更新函数
         * @param P0 初始状态协方差矩阵
         * @param X0 初始状态向量
         * @param x_add_ 状态加法函数
         * @details 完整初始化滤波器，设置所有必要的函数和初始条件
         */
        ExtendedKalman(const Func_x1d_x1d &f_, const Func_z1d_x1d &h_,
                       const Func_xxd_x1d &cal_F_, const Func_zxd_x1d cal_H_,
                       const Func_xxd_x1d &update_Q_, const Func_zzd_z1d &update_R_,
                       const Matrix_xxd &P0, const Matrix_x1d &X0, const Add_func &x_add_)
        : f(f_), h(h_), calculate_F(cal_F_), calculate_H(cal_H_), update_Q(update_Q_),
            update_R(update_R_), P_post(P0), X_post(X0), x_add(x_add_)
        {
        }
        
        /**
         * @brief 重置滤波器状态
         * @param X0 新的初始状态向量
         * @details 仅重置状态向量，保持其他参数不变
         */
        void reset(const Matrix_x1d &X0) 
        {
            X_post = X0;
        }

        /**
         * @brief 重置状态向量的特定元素
         * @param index 要重置的状态元素索引
         * @param value 新的状态值
         * @details 用于单独修改状态向量中的某个分量
         */
        void reset(const int &index, const double value)
        {
            X_post(index, 0) = value;
        }

        /**
         * @brief 完全重置滤波器 - 重新设置所有参数
         * @param f_ 状态转移函数
         * @param h_ 观测函数
         * @param cal_F_ 状态雅可比计算函数
         * @param cal_H_ 观测雅可比计算函数
         * @param update_Q_ 过程噪声协方差更新函数
         * @param update_R_ 观测噪声协方差更新函数
         * @param P0 初始状态协方差矩阵
         * @param X0 初始状态向量
         * @param x_add_ 状态加法函数
         * @details 重新初始化滤波器的所有参数，相当于重新构造
         */
        void reset(const Func_x1d_x1d &f_, const Func_z1d_x1d &h_,
                   const Func_xxd_x1d &cal_F_, const Func_zxd_x1d cal_H_,
                   const Func_xxd_x1d &update_Q_, const Func_zzd_z1d &update_R_,
                   const Matrix_xxd &P0, const Matrix_x1d &X0, const Add_func &x_add_) 
        {
            f = f_;
            h = h_;
            calculate_F = cal_F_;
            calculate_H = cal_H_;
            update_Q = update_Q_;
            update_R = update_R_;
            P_post = P0;
            X_post = X0;
            x_add = x_add_;
        }

        /**
         * @brief 预测步骤 - 计算先验状态估计
         * @return Matrix_x1d 先验状态估计 x⁻(k+1|k)
         * @details 执行EKF的预测步骤：
         *          1. 计算状态雅可比矩阵 F = ∂f/∂x
         *          2. 更新过程噪声协方差矩阵 Q
         *          3. 状态预测：x⁻ = f(x⁺)
         *          4. 协方差预测：P⁻ = F·P⁺·Fᵀ + Q
         */
        Matrix_x1d predict()
        {
            // 计算当前状态的雅可比矩阵
            F = calculate_F(X_post);

            // 更新过程噪声协方差矩阵
            Q = update_Q(X_post);

            // 状态预测
            X_pri = f(X_post);
            
            // 协方差预测
            P_pri = F * P_post * F.transpose() + Q;

            // 更新当前估计为先验估计
            X_post = X_pri;
            P_post = P_pri;

            return X_pri;
        }

        /**
         * @brief 更新步骤 - 融合观测信息得到后验估计
         * @param Z 当前时刻的观测向量
         * @return Matrix_x1d 后验状态估计 x⁺(k+1|k+1)
         * @details 执行EKF的更新步骤：
         *          1. 计算观测雅可比矩阵 H = ∂h/∂x
         *          2. 更新观测噪声协方差矩阵 R
         *          3. 计算卡尔曼增益：K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
         *          4. 状态更新：x⁺ = x⁻ + K·(z - h(x⁻))
         *          5. 协方差更新：P⁺ = (I - K·H)·P⁻
         */
        Matrix_x1d update(const Matrix_z1d &Z)
        {
            // 计算观测雅可比矩阵
            H = calculate_H(X_pri);

            // 更新观测噪声协方差矩阵
            R = update_R(Z);

            // 计算卡尔曼增益
            K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
            
            // 状态更新
            X_post = x_add(X_pri, K * (Z - h(X_pri)));
            
            // 协方差更新
            P_post = (Matrix_xxd::Identity() - K * H) * P_pri;

            return X_post;
        }
    }; // class ExtendedKalman

} // namespace predict

#endif /* _PREDICT_EXTENDED_KALMAN_HPP_ */

