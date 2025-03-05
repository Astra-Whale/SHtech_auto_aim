#ifndef _EXTENDEDKALMAN_H_
#define _EXTENDEDKALMAN_H_

#include <Eigen/Dense>
#include <chrono>
#include <functional>

using namespace std::chrono;

template <int V_Z, int V_X> //<观测维度，状态维度>
class ExtendedKalman
{
public:
    using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
    using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
    using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
    using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
    using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
    using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;
    using TP = std::chrono::high_resolution_clock::time_point;

private:
    Matrix_x1d x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Matrix_xzd K;    // Kalman增益
    Matrix_xxd R;    // 预测过程噪声偏差的方差
    Matrix_zzd Q;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    Matrix_xxd P;    // 估计误差协方差
    TP last_tp;

    // 非线性状态转移函数和观测函数
    std::function<Matrix_x1d(const Matrix_x1d &, double)> f;
    std::function<Matrix_z1d(const Matrix_x1d &)> h;

    // 雅可比矩阵计算函数
    std::function<Matrix_xxd(const Matrix_x1d &, double)> F_jacobian;
    std::function<Matrix_zxd(const Matrix_x1d &)> H_jacobian;

public:
    ExtendedKalman() = default;

    ExtendedKalman(Matrix_xxd R, Matrix_zzd Q, Matrix_x1d init, TP last_time,
                   std::function<Matrix_x1d(const Matrix_x1d &, double)> f, std::function<Matrix_z1d(const Matrix_x1d &)> h,
                   std::function<Matrix_xxd(const Matrix_x1d &, double)> F_jacobian, std::function<Matrix_zxd(const Matrix_x1d &)> H_jacobian)
    {
        reset(R, Q, init, last_time, f, h, F_jacobian, H_jacobian);
    }

    void reset(Matrix_xxd R, Matrix_zzd Q, Matrix_x1d init, TP last_time,
               std::function<Matrix_x1d(const Matrix_x1d &, double)> f, std::function<Matrix_z1d(const Matrix_x1d &)> h,
               std::function<Matrix_xxd(const Matrix_x1d &, double)> F_jacobian, std::function<Matrix_zxd(const Matrix_x1d &)> H_jacobian)
    {
        this->P = Matrix_xxd::Zero();
        this->R = R;
        this->Q = Q;
        this->f = f;
        this->h = h;
        this->F_jacobian = F_jacobian;
        this->H_jacobian = H_jacobian;
        x_k1 = init;
        last_tp = last_time;
    }

    void reset(Matrix_x1d init, TP last_time)
    {
        x_k1 = init;
        last_tp = last_time;
    }

    void reset(double x, TP last_time)
    {
        x_k1(0, 0) = x;
        last_tp = last_time;
    }

    // 根据输入时间t进行预测,不包含观测更新
    Matrix_x1d state_predict_only(TP predict_time) 
    {
        // 计算预测时间步长
        double dt = duration_cast<microseconds>(predict_time - last_tp).count() / 1e6;
        
        // 仅执行预测步骤
        Matrix_x1d predicted_state = f(x_k1, dt);
        
        // 注意:这里不更新last_tp,因为这只是预测
        // 也不更新状态向量x_k1,因为没有观测值来校正预测
        
        return predicted_state;
    }

    // 根据直接输入的时间确定delay后的状态
    Matrix_x1d state_predict_only(double delay)  // t_delay单位：秒
    {
        Matrix_x1d predicted_state = f(x_k1, delay);
        return predicted_state;
    }

    Matrix_x1d update(Matrix_z1d z_k, TP time)
    {
        // 计算时间步长
        double dt = duration_cast<microseconds>(time - last_tp).count() / 1e6;
        last_tp = time;

        // 预测下一时刻的值
        Matrix_x1d p_x_k = f(x_k1, dt);

        // 计算雅可比矩阵
        Matrix_xxd F_j = F_jacobian(x_k1, dt);

        // 求协方差
        P = F_j * P * F_j.transpose() + R;

        // 计算观测雅可比矩阵
        Matrix_zxd H_j = H_jacobian(p_x_k);

        // 计算Kalman增益
        K = P * H_j.transpose() * (H_j * P * H_j.transpose() + Q).inverse();

        // 修正结果，即计算滤波值
        x_k1 = p_x_k + K * (z_k - h(p_x_k));

        // 更新后验估计
        P = (Matrix_xxd::Identity() - K * H_j) * P;

        return x_k1;
    }

    Matrix_x1d update_with_debug_print(Matrix_z1d z_k, TP time)
    {
        // 计算时间步长
        double dt = duration_cast<microseconds>(time - last_tp).count() / 1e6;
        last_tp = time;

        std::cout << "\n============= EKF Update Information =============\n";
        std::cout << "Time Step: " << std::fixed << std::setprecision(3) << dt << " s\n";

        // 预测步骤
        Matrix_x1d p_x_k = f(x_k1, dt);
        
        // 打印预测信息
        std::cout << "\n=== Prediction Step ===\n";
        std::cout << "Previous State:\n";
        std::cout << "Position: (" << x_k1(0) << ", " << x_k1(1) << "), Yaw: " << x_k1(2) << "\n";
        std::cout << "Predicted State:\n";
        std::cout << "Position: (" << p_x_k(0) << ", " << p_x_k(1) << "), Yaw: " << p_x_k(2) << "\n";

        // 计算雅可比矩阵
        Matrix_xxd F_j = F_jacobian(x_k1, dt);
        Matrix_zxd H_j = H_jacobian(p_x_k);

        // 预测观测值
        Matrix_z1d predicted_z = h(p_x_k);
        Matrix_z1d innovation = z_k - predicted_z;

        // 打印观测对比信息
        std::cout << "\n=== Measurement vs Prediction ===\n";
        std::cout << "Actual Measurement:  (" << z_k(0) << ", " << z_k(1) << ", " << z_k(2) << ")\n";
        std::cout << "Predicted Measurement: (" << predicted_z(0) << ", " << predicted_z(1) 
                << ", " << predicted_z(2) << ")\n";
        std::cout << "Innovation: (" << innovation(0) << ", " << innovation(1) 
                << ", " << innovation(2) << ")\n";

        // 求协方差
        P = F_j * P * F_j.transpose() + R;

        // 计算Kalman增益
        Matrix_zzd S = H_j * P * H_j.transpose() + Q;  // Innovation covariance
        K = P * H_j.transpose() * S.inverse();

        // 打印协方差和增益信息
        std::cout << "\n=== Filter Parameters ===\n";
        std::cout << "Max Innovation Covariance: " << S.maxCoeff() << "\n";
        std::cout << "Min Innovation Covariance: " << S.minCoeff() << "\n";
        std::cout << "Max Kalman Gain: " << K.maxCoeff() << "\n";
        std::cout << "Min Kalman Gain: " << K.minCoeff() << "\n";

        // 修正结果
        Matrix_x1d state_correction = K * innovation;
        x_k1 = p_x_k + state_correction;

        // 打印状态修正信息
        std::cout << "\n=== State Correction ===\n";
        std::cout << "Correction:\n";
        std::cout << "Position: (" << state_correction(0) << ", " << state_correction(1) 
                << "), Yaw: " << state_correction(2) << "\n";
        std::cout << "Final State:\n";
        std::cout << "Position: (" << x_k1(0) << ", " << x_k1(1) << "), Yaw: " << x_k1(2) << "\n";

        // 更新后验估计
        P = (Matrix_xxd::Identity() - K * H_j) * P;

        std::cout << "\n=== Uncertainties ===\n";
        std::cout << "Position uncertainty: " << std::sqrt(P(0,0)) << ", " << std::sqrt(P(1,1)) << "\n";
        std::cout << "Yaw uncertainty: " << std::sqrt(P(2,2)) << "\n";

        std::cout << "=============================================\n\n";

        return x_k1;
    }

    Matrix_x1d get_state()
    {
        return x_k1;
    }

    TP get_last_tp()
    {
        return last_tp;
    }

    double get_duration(TP time)
    {
        return duration_cast<microseconds>(time - last_tp).count() / 1e6;
    }

    // 警戒！这涉及直接写入状态向量。
    // 目前，它仅被允许在切换装甲板时被调用。
    void directly_change_state(Matrix_x1d new_state)
    {
        x_k1 = new_state; // 更改状态量
    }

    void printStateInfo() const {
        // 获取当前状态向量和时间戳
        Matrix_x1d state = x_k1;
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_tp).count();
        
        // 计算装甲板速度
        double armor_vx = state(3) - state(6) * state(5) * sin(state(2));
        double armor_vy = state(4) + state(6) * state(5) * cos(state(2));
        double v_magnitude = std::sqrt(armor_vx * armor_vx + armor_vy * armor_vy);
        
        // 获取系统时间用于显示
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);
        
        std::cout << "\n============= EKF State Information =============\n";
        
        // 时间信息
        std::cout << "Time: " << std::ctime(&now_time);
        std::cout << "Time Since Last Update: " << time_since_last << " ms\n";
        
        // 状态信息
        std::cout << "\n--- State Vector ---\n";
        std::cout << "Position (x, y): (" 
                << std::fixed << std::setprecision(3) 
                << state(0) << ", " << state(1) << ") m\n";
                
        std::cout << "Yaw Angle: " 
                << std::fixed << std::setprecision(1) 
                << state(2) << " deg\n";
                
        std::cout << "Linear Velocity (vx, vy): (" 
                << std::fixed << std::setprecision(3) 
                << state(3) << ", " << state(4) << ") m/s\n";
                
        std::cout << "Angular Velocity: " 
                << std::fixed << std::setprecision(1) 
                << state(5) << " deg/s\n";
                
        std::cout << "Armor Radius: " 
                << std::fixed << std::setprecision(3) 
                << state(6) << " m\n";
        
        // 装甲板衍生信息
        std::cout << "\n--- Derived Armor Information ---\n";
        std::cout << "Armor Velocity (vx, vy): (" 
                << std::fixed << std::setprecision(3) 
                << armor_vx << ", " << armor_vy << ") m/s\n";
        std::cout << "Armor Speed Magnitude: " 
                << std::fixed << std::setprecision(3) 
                << v_magnitude << " m/s\n";
                
        // 协方差矩阵信息
        std::cout << "\n--- Covariance Information ---\n";
        std::cout << "Position Uncertainty (x, y): (" 
                << std::fixed << std::setprecision(3) 
                << std::sqrt(P(0,0)) << ", " << std::sqrt(P(1,1)) << ") m\n";
        std::cout << "Yaw Uncertainty: " 
                << std::fixed << std::setprecision(1) 
                << std::sqrt(P(2,2)) << " deg\n";
        std::cout << "Velocity Uncertainty (vx, vy): (" 
                << std::fixed << std::setprecision(3) 
                << std::sqrt(P(3,3)) << ", " << std::sqrt(P(4,4)) << ") m/s\n";
        std::cout << "Angular Velocity Uncertainty: " 
                << std::fixed << std::setprecision(1) 
                << std::sqrt(P(5,5)) << " deg/s\n";
        std::cout << "Radius Uncertainty: " 
                << std::fixed << std::setprecision(3) 
                << std::sqrt(P(6,6)) << " m\n";
        
        // Kalman增益信息
        std::cout << "\n--- Kalman Gain Matrix ---\n";
        std::cout << "Latest Kalman Gain:\n";
        for(int i = 0; i < V_X; i++) {
            for(int j = 0; j < V_Z; j++) {
                std::cout << std::fixed << std::setprecision(4) << K(i,j) << " ";
            }
            std::cout << "\n";
        }
        
        std::cout << "\n============================================\n";
    }
};

#endif /* _EXTENDEDKALMAN_H_ */
