/**
 * @file CoordTransformer.hpp
 * @brief 坐标变换模块 - 处理相机、IMU和世界坐标系之间的转换与PnP求解
 * @author Cao Jingyan
 * @date 2025/11/15
 * 
 * 该模块负责：
 * 1. 管理相机内参和外参
 * 2. 执行PnP算法进行位姿估计
 * 3. 在不同坐标系之间进行坐标变换
 * 4. 计算装甲板的空间位置和姿态
 */

#ifndef PREDICT_COORD_TRANSFORMER_H
#define PREDICT_COORD_TRANSFORMER_H

// modules
#include "common.hpp"

// packages
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    /// @brief 三维位置向量类型别名，使用Eigen::Vector3d
    using Pos3D = Eigen::Vector3d;

    /**
     * @brief 自定义坐标系到PnP坐标系的转换矩阵
     * @details 该矩阵用于将自定义的世界坐标系转换为PnP算法使用的标准坐标系
     *          实现坐标系的翻转和轴交换
     */
    const Eigen::Matrix3d R_custom2pnp = 
        (Eigen::Matrix3d() << -1., 0., 0.,
                        0., 0., 1.,
                        0., 1., 0. ).finished();

    /**
     * @brief 小装甲板的3D模型坐标点
     * @details 定义小装甲板四个角点在装甲板本地坐标系中的位置（单位：米）
     *          角点顺序：左上、左下、右下、右上
     *          尺寸：13.5cm × 5.6cm
     */
    const std::vector<cv::Point3d> pw_small = {
                {-0.0675, -0.028, 0.},  // 左上角点
                {-0.0675, 0.028, 0.},   // 左下角点
                {0.0675, 0.028, 0.},    // 右下角点
                {0.0675, -0.028, 0.}};  // 右上角点
    
    /**
     * @brief 大装甲板的3D模型坐标点
     * @details 定义大装甲板四个角点在装甲板本地坐标系中的位置（单位：米）
     *          角点顺序：左上、左下、右下、右上
     *          尺寸：23.0cm × 5.8cm
     */
    const std::vector<cv::Point3d> pw_big = {
                {-0.115, -0.029, 0.},   // 左上角点
                {-0.115, 0.029, 0.},    // 左下角点
                {0.115, 0.029, 0.},     // 右下角点
                {0.115, -0.029, 0.}};   // 右上角点

    /**
     * @class CoordTransformer
     * @brief 坐标变换器类 - 负责多坐标系间的变换和PnP位姿估计
     * @details 该类管理相机参数，执行PnP算法，并提供各种坐标系之间的变换功能
     *          支持的坐标系：
     *          - 相机坐标系 (Camera): 相机光心为原点，使用opencv2::solvepnp使用的坐标系
     *          - IMU坐标系 (IMU): 云台旋转点为原点的坐标系，IMU给出云台姿态数据，使用opencv2::solvepnp使用的坐标系
     *          - 世界坐标系 (World): 全局坐标系，以云台旋转点为原点的坐标系，欧拉角固定为0
     *          - 图像坐标系 (Image): 图像像素坐标系
     */
    class CoordTransformer
    {
    private:
        // === 坐标变换参数 (Eigen格式，用于数学计算) ===
        /// @brief 相机到IMU的平移向量 [3x1]
        Eigen::Vector3d T_camera2imu;
        
        /// @brief 相机到IMU的旋转矩阵 [3x3]
        Eigen::Matrix3d R_camera2imu;
        
        /// @brief 相机内参矩阵 [3x3] - 包含焦距fx,fy和主点坐标cx,cy
        Eigen::Matrix3d F;
        
        /// @brief 相机畸变参数 [1x5] - 径向畸变k1,k2,k3和切向畸变p1,p2
        Eigen::Matrix<double, 1, 5> C;
        
        // === 坐标变换参数 (OpenCV格式，用于文件读写和PnP算法) ===
        /// @brief 相机到IMU的平移向量 (OpenCV Mat格式)
        cv::Mat T_camera2imu_MAT;
        
        /// @brief 相机到IMU的旋转矩阵 (OpenCV Mat格式)
        cv::Mat R_camera2imu_MAT;
        
        /// @brief 相机内参矩阵 (OpenCV Mat格式)
        cv::Mat F_MAT;
        
        /// @brief 相机畸变参数 (OpenCV Mat格式)
        cv::Mat C_MAT;

        /// @brief 世界坐标系到IMU坐标系的旋转矩阵 - 由IMU姿态实时更新
        Eigen::Matrix3d R_world2imu; 
                      
    public:
        /**
         * @brief 默认构造函数
         */
        explicit CoordTransformer();
        
        /**
         * @brief 带参数构造函数
         * @param camera_param 相机参数文件路径
         * @details 从YAML配置文件中读取相机内参、畸变参数和外参
         *          文件应包含以下字段：
         *          - T_c2i: 相机到IMU的平移向量
         *          - R_c2i: 相机到IMU的旋转矩阵
         *          - K: 相机内参矩阵
         *          - D: 相机畸变参数
         */
        explicit CoordTransformer(const std::string camera_param);

        /**
         * @brief 更新世界坐标系到IMU坐标系的旋转矩阵
         * @param q_raw 从IMU姿态得到的四元数
         * @details 在每次接收到新的IMU数据时调用，更新实时的坐标变换关系
         *          该函数会结合预定义的坐标系转换矩阵R_custom2pnp
         */
        void update_R_world2imu(const Eigen::Quaternionf &q_raw);

        /**
         * @brief PnP算法获取装甲板测量值
         * @param p 装甲板四个角点的图像像素坐标数组 [4] (按顺序：左上、左下、右下、右上)
         * @param armor_number 装甲板编号 (0,1,8=大装甲板，其他=小装甲板)
         * @param attitude_yaw 机器人当前姿态偏航角 (弧度)
         * @param yaw_in_camera [输出] 装甲板在相机坐标系中的偏航角 (弧度)
         * @return Eigen::Vector4d 装甲板测量值 [y, x, z, absolute_yaw]
         *         - x,y,z: 装甲板中心在世界坐标系中的位置 (米)
         *         - absolute_yaw: 装甲板绝对偏航角 = 相机系偏航角 - 机器人偏航角
         * @details 通过PnP算法从2D像素点反推装甲板的3D位置和姿态
         *          包括坐标变换、法向量计算和角度解算
         */
        Eigen::Vector4d pnp_get_measurement(const cv::Point2f (&p)[4], const int &armor_number, const float &attitude_yaw, float &yaw_in_camera);

        // === 坐标变换内联函数 ===
        /**
         * @brief 相机坐标系 → 世界坐标系坐标变换
         * @param pc 相机坐标系中的3D点
         * @return Pos3D 世界坐标系中的3D点
         * @details 变换链：Camera → IMU → World
         *          pc -(R_camera2imu,T_camera2imu)-> pi -(R_world2imu^T)-> pw
         */
        inline Pos3D pc_to_pw(const Pos3D &pc)
        {
            return R_world2imu.transpose() * (R_camera2imu * pc + T_camera2imu);
        }

        /**
         * @brief 世界坐标系 → 相机坐标系坐标变换
         * @param pw 世界坐标系中的3D点
         * @return Pos3D 相机坐标系中的3D点
         * @details 变换链：World → IMU → Camera
         *          pw -(R_world2imu)-> pi -(R_camera2imu^T,T_camera2imu)-> pc
         */
        inline Pos3D pw_to_pc(const Pos3D &pw)
        {
            return R_camera2imu.transpose() * (R_world2imu * pw - T_camera2imu);
        }

        /**
         * @brief 世界坐标系 → IMU坐标系坐标变换
         * @param pw 世界坐标系中的3D点
         * @return Pos3D IMU坐标系中的3D点
         * @details 直接通过旋转矩阵变换，不包含平移
         */
        inline Pos3D pw_to_pi(const Pos3D &pw)
        {
            return R_world2imu * pw;
        }

        /**
         * @brief 相机坐标系 → 图像像素坐标系投影变换
         * @param pc 相机坐标系中的3D点
         * @return Pos3D 图像坐标系中的齐次坐标 [u, v, 1]
         * @details 通过相机内参矩阵F进行透视投影变换
         *          投影公式：[u, v, w]^T = F * [X, Y, Z]^T, 像素坐标 = [u/w, v/w]
         *          注意：返回值需要归一化 (除以Z坐标) 得到真实像素坐标
         */
        inline Pos3D pc_to_pu(const Pos3D &pc)
        {
            return F * pc / pc(2, 0);
        }
        
    }; // class CoordTransformer

} // namespace predict

#endif // PREDICT_COORD_TRANSFORMER_H
