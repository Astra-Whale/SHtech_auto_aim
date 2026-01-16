/**
 * @file IESEKF.hpp
 * @brief 迭代扩展卡尔曼滤波器 (IESEKF) 模块
 * @author Cao Jingyan
 * @date 2026/01/17
 * 
 * 该模块提供：
 * 1. 迭代误差状态扩展卡尔曼滤波器实现
 * 2. 相比普通EKF，增加了高斯-牛顿迭代更新过程
 * 3. 显著解决高速旋转和强非线性系统下的线性化误差问题
 * 4. 支持流形运算（角度归一化等）
 * 
 * 核心改进：
 * - 在更新阶段进行多次迭代，不断寻找最优线性化工作点
 * - 引入 x_minus 接口处理角度等周期性状态量
 * - 提高对急停、变向、变速旋转等复杂运动的跟踪精度
 */

#ifndef _PREDICT_IESEKF_HPP_
#define _PREDICT_IESEKF_HPP_

#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include <cmath>

namespace mathutils
{
    /**
     * @class IESEKF
     * @brief 迭代误差状态扩展卡尔曼滤波器
     * @tparam V_Z 观测向量维度
     * @tparam V_X 状态向量维度
     * @details 通过在更新阶段多次迭代，不断在当前最佳估计点处重新线性化
     *          从而获得更准确的后验估计，特别适合强非线性系统
     */
    template <int V_Z = 4, int V_X = 9>
    class IESEKF
    {
    public:
        // === 类型定义 ===
        using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
        using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
        using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
        using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
        using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
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
        
        /// @brief 广义加法函数类型：x_new = x + delta
        using Add_func = std::function<Matrix_x1d(const Matrix_x1d &, const Matrix_x1d &)>;
        
        /// @brief 广义减法函数类型：delta = x1 - x2 (用于处理角度差等流形运算)
        using Minus_func = std::function<Matrix_x1d(const Matrix_x1d &, const Matrix_x1d &)>;

    private:
        // === 系统方程 ===
        Func_x1d_x1d f;
        Func_z1d_x1d h;
        Func_xxd_x1d calculate_F;
        Func_zxd_x1d calculate_H;
        Func_xxd_x1d update_Q;
        Func_zzd_z1d update_R;
        Add_func x_add;
        Minus_func x_minus;

        // === 矩阵存储 ===
        Matrix_xxd F;
        Matrix_zxd H;
        Matrix_xxd Q;
        Matrix_zzd R;
        Matrix_xxd P_pri;
        Matrix_xxd P_post;
        Matrix_xzd K;
        Matrix_x1d X_pri;
        Matrix_x1d X_post;

        // === 迭代参数 ===
        int max_iter;
        double stop_threshold;

    public:
        /**
         * @brief 默认构造函数
         */
        IESEKF() = default;

        /**
         * @brief 构造函数 - 初始化IESEKF
         * @param f_ 状态转移函数
         * @param h_ 观测函数
         * @param cal_F_ 状态雅可比计算函数
         * @param cal_H_ 观测雅可比计算函数
         * @param update_Q_ 过程噪声协方差更新函数
         * @param update_R_ 观测噪声协方差更新函数
         * @param P0 初始状态协方差矩阵
         * @param X0 初始状态向量
         * @param x_add_ 状态加法函数
         * @param x_minus_ 状态减法函数（处理流形运算）
         * @param max_iter_ 最大迭代次数，建议 3-5 次
         * @param stop_threshold_ 收敛阈值（状态修正量的范数），建议 1e-4
         */
        IESEKF(const Func_x1d_x1d &f_, const Func_z1d_x1d &h_,
               const Func_xxd_x1d &cal_F_, const Func_zxd_x1d cal_H_,
               const Func_xxd_x1d &update_Q_, const Func_zzd_z1d &update_R_,
               const Matrix_xxd &P0, const Matrix_x1d &X0, 
               const Add_func &x_add_, const Minus_func &x_minus_,
               int max_iter_ = 5, double stop_threshold_ = 1e-4)
        : f(f_), h(h_), calculate_F(cal_F_), calculate_H(cal_H_), update_Q(update_Q_),
          update_R(update_R_), P_post(P0), X_post(X0), 
          x_add(x_add_), x_minus(x_minus_),
          max_iter(max_iter_), stop_threshold(stop_threshold_)
        {
        }

        /**
         * @brief 重置状态向量
         * @param X0 新的初始状态向量
         */
        void reset(const Matrix_x1d &X0) 
        { 
            X_post = X0; 
        }

        /**
         * @brief 完整重置滤波器
         * @param f_ 状态转移函数
         * @param h_ 观测函数
         * @param cal_F_ 状态雅可比计算函数
         * @param cal_H_ 观测雅可比计算函数
         * @param update_Q_ 过程噪声协方差更新函数
         * @param update_R_ 观测噪声协方差更新函数
         * @param P0 初始状态协方差矩阵
         * @param X0 初始状态向量
         * @param x_add_ 状态加法函数
         * @param x_minus_ 状态减法函数
         */
        void reset(const Func_x1d_x1d &f_, const Func_z1d_x1d &h_,
                   const Func_xxd_x1d &cal_F_, const Func_zxd_x1d cal_H_,
                   const Func_xxd_x1d &update_Q_, const Func_zzd_z1d &update_R_,
                   const Matrix_xxd &P0, const Matrix_x1d &X0, 
                   const Add_func &x_add_, const Minus_func &x_minus_) 
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
            x_minus = x_minus_;
        }

        /**
         * @brief 预测步骤 (与标准 EKF 相同)
         * @return 先验状态估计
         * @details 根据状态转移方程和过程噪声预测下一时刻状态
         */
        Matrix_x1d predict()
        {
            F = calculate_F(X_post);
            Q = update_Q(X_post);
            X_pri = f(X_post);
            P_pri = F * P_post * F.transpose() + Q;
            X_post = X_pri;
            P_post = P_pri;
            return X_pri;
        }

        /**
         * @brief 迭代更新步骤 (IESEKF 核心算法)
         * @param Z 当前观测值
         * @return 后验状态估计
         * @details 通过多次迭代，在不同工作点处重新线性化观测方程
         *          每次迭代都计算：
         *          1. 在当前迭代点处的观测雅可比 H
         *          2. 计算卡尔曼增益 K
         *          3. 计算观测残差和先验残差
         *          4. 更新状态估计
         *          5. 检查收敛条件
         */
        Matrix_x1d update(const Matrix_z1d &Z)
        {
            R = update_R(Z);
            
            // 迭代初始点设为预测值 (先验)
            Matrix_x1d X_cur = X_pri; 

            for (int i = 0; i < max_iter; ++i)
            {
                // 1. 在当前迭代点处重新线性化
                H = calculate_H(X_cur);
                
                // 2. 计算卡尔曼增益
                // K = P_pri * H^T * (H * P_pri * H^T + R)^-1
                Matrix_zzd S = H * P_pri * H.transpose() + R;
                K = P_pri * H.transpose() * S.inverse();

                // 3. 计算残差
                // 观测残差: z - h(x_k)
                Matrix_z1d innovation = Z - h(X_cur);
                
                // 先验残差: x_pri - x_k (使用 x_minus 处理角度归一化)
                Matrix_x1d dx_pri = x_minus(X_pri, X_cur);

                // 4. 计算状态修正量
                // IESEKF 更新公式: x_{k+1} = x_pri + K * (innovation + H * dx_pri)
                // 这一步将当前工作点拉向"观测"和"先验"的平衡点
                Matrix_x1d dx = K * (innovation + H * dx_pri);
                
                // 得到下一次迭代的状态
                Matrix_x1d X_next = x_add(X_pri, dx);

                // 5. 收敛判断
                // 比较 X_next 和 X_cur 的差异
                Matrix_x1d diff = x_minus(X_next, X_cur);
                if (diff.norm() < stop_threshold) {
                    X_cur = X_next;
                    break;
                }
                X_cur = X_next;
            }

            // 更新最终状态
            X_post = X_cur;
            
            // 更新协方差 (使用最后一次线性化的 H)
            // 注意：这里使用 Joseph 形式会更稳定，但计算量更大
            P_post = (Matrix_xxd::Identity() - K * H) * P_pri;

            return X_post;
        }

        /**
         * @brief 获取当前状态协方差矩阵
         * @return 后验状态协方差矩阵
         */
        Matrix_xxd get_P() const { return P_post; }

        /**
         * @brief 获取当前状态估计
         * @return 后验状态向量
         */
        Matrix_x1d get_X() const { return X_post; }

        /**
         * @brief 设置迭代参数
         * @param max_iter_ 最大迭代次数
         * @param stop_threshold_ 收敛阈值
         */
        void set_iteration_params(int max_iter_, double stop_threshold_)
        {
            max_iter = max_iter_;
            stop_threshold = stop_threshold_;
        }
    }; 

} // namespace mathutils

#endif // _PREDICT_IESEKF_HPP_
