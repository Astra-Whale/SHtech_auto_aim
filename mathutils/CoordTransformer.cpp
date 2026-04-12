/**
 * @file CoordTransformer.cpp
 * @brief 坐标变换模块实现 - 处理相机、IMU和世界坐标系之间的转换
 * @author Cao Jingyan
 * @date 2025/11/15
 */

#include "CoordTransformer.hpp"

namespace mathutils
{
    // 定义静态成员变量
    std::unique_ptr<CoordTransformer> CoordTransformer::instance_ = nullptr;

    /**
     * @brief 默认构造函数 - 初始化所有变换矩阵为零矩阵
     */
    CoordTransformer::CoordTransformer(bool adjust_)
    {
        // 初始化相机到IMU的平移向量 (3x1)
        T_camera2imu_MAT = cv::Mat::zeros(3, 1, CV_64FC1);
        
        // 初始化相机到IMU的旋转矩阵 (3x3)
        R_camera2imu_MAT = cv::Mat::zeros(3, 3, CV_64FC1);
        
        // 初始化相机内参矩阵 (3x3)
        F_MAT = cv::Mat::zeros(3, 3, CV_64FC1);
        
        // 初始化相机畸变参数 (1x5)
        C_MAT = cv::Mat::zeros(1, 5, CV_64FC1);
        
        // 将OpenCV矩阵转换为Eigen矩阵，便于后续数学运算
        cv::cv2eigen(T_camera2imu_MAT, T_camera2imu);
        cv::cv2eigen(R_camera2imu_MAT, R_camera2imu);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        adjust = adjust_;

        if (adjust) {
            cv::namedWindow("transformer trackbar", cv::WINDOW_AUTOSIZE);
        
            cv::createTrackbar("pw_length", "transformer trackbar", &pw_length, 250, 0);
            cv::createTrackbar("pw_width", "transformer trackbar", &pw_width, 250, 0);
        }
    }

    /**
     * @brief 带参数构造函数 - 从yml文件加载相机参数
     * @param camera_param yml文件路径
     * @details 从YAML文件中读取相机内参、畸变参数和相机-IMU外参
     */
    CoordTransformer::CoordTransformer(const std::string camera_param, bool adjust_)
    {
        // 打开相机参数文件
        cv::FileStorage fin(camera_param, cv::FileStorage::READ);
        
        // 读取相机到IMU的外参（平移向量和旋转矩阵）
        fin["T_c2i"] >> T_camera2imu_MAT;  // 相机到IMU的平移向量
        fin["R_c2i"] >> R_camera2imu_MAT;  // 相机到IMU的旋转矩阵

        // 读取相机内参
        fin["K"] >> F_MAT;  // 相机内参矩阵 (焦距、主点等)
        fin["D"] >> C_MAT;  // 相机畸变参数 (径向畸变、切向畸变)

        // 将OpenCV格式转换为Eigen格式，便于数学运算
        cv::cv2eigen(T_camera2imu_MAT, T_camera2imu);
        cv::cv2eigen(R_camera2imu_MAT, R_camera2imu);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        adjust = adjust_;

        if (adjust) {
            cv::namedWindow("transformer trackbar", cv::WINDOW_AUTOSIZE);
        
            cv::createTrackbar("pw_length", "transformer trackbar", &pw_length, 250, 0);
            cv::createTrackbar("pw_width", "transformer trackbar", &pw_width, 250, 0);
        }
    }

    /**
     * @brief 显式初始化单例
     */
    void CoordTransformer::Init(const std::string& param_file, bool adjust)
    {
        if (!instance_) {
            instance_ = std::unique_ptr<CoordTransformer>(new CoordTransformer(param_file, adjust));
        }
    }

    /**
     * @brief 获取单例实例
     */
    CoordTransformer& CoordTransformer::Get()
    {
        if (!instance_) {
            // 防御性编程：如果未初始化就调用，直接报错终止
            std::cerr << "[FATAL] CoordTransformer accessed before Init!" << std::endl;
            std::abort();
        }
        return *instance_;
    }

    /**
     * @brief 销毁单例
     */
    void CoordTransformer::Destroy()
    {
        instance_.reset();
    }

    /**
     * @brief PnP算法获取装甲板测量值
     * @param p 装甲板四个角点的图像像素坐标 (按顺序：左上、左下、右下、右上)
     * @param armor_number 装甲板编号 (0,1,8为大装甲板，其他为小装甲板)
     * @param color_id 颜色ID
     * @param attitude_yaw 机器人当前姿态的偏航角 (弧度)
     * @param R_world2imu 世界坐标系到IMU坐标系的旋转矩阵
     * @param yaw_in_camera 输出参数：装甲板在相机坐标系中的偏航角
     * @param measurement 输出参数：装甲板的测量值 [y, x, z, absolute_yaw]
     * @return bool 成功标志，true表示PnP求解成功，false表示失败
     * @details 通过PnP算法从2D图像点反推3D世界坐标，并计算装甲板朝向
     */
    bool CoordTransformer::pnp_get_measurement(const cv::Point2f (&p)[4], const int &armor_number, 
                                                const int &color_id, const float &attitude_yaw, 
                                                const Eigen::Matrix3d &R_world2imu,
                                                float &yaw_in_camera, Eigen::Vector4d &measurement)
    {
        // 根据装甲板编号选择对应的3D模型尺寸
        std::vector<cv::Point3d> pw_cur;
        if (adjust) {
            double pw_x = pw_length / 1000.0 / 2.0;
            double pw_y = pw_width / 1000.0 / 2.0;

            std::vector<cv::Point3d> pw_temp = {
                {-pw_x, -pw_y, 0.},  // 左上角点
                {-pw_x, pw_y, 0.},   // 左下角点
                {pw_x, pw_y, 0.},    // 右下角点
                {pw_x, -pw_y, 0.}};  // 右上角点

            pw_cur = pw_temp;
        }
        else {
            if (color_id == 0) {
                if (armor_number == 0 || armor_number == 1 || armor_number == 8)
                    pw_cur = pw_red_big;    // 大装甲板
                else
                    pw_cur = pw_red_small;  // 小装甲板
            }
            else {
                if (armor_number == 0 || armor_number == 1 || armor_number == 8)
                    pw_cur = pw_blue_big;    // 大装甲板
                else
                    pw_cur = pw_blue_small;  // 小装甲板
            }
        }

        // for (int i = 0; i < 4; ++i) {
        //     std::cout << p[i] << std::endl;
        // }

        // std::cout << "pw_cur: " << std::endl;
        // for (const auto &pt : pw_cur) {
        //     std::cout << pt << std::endl;
        // }

        // std::cout << "F_MAT: " << std::endl << F_MAT << std::endl;
        // std::cout << "C_MAT: " << std::endl << C_MAT << std::endl;

        // 将图像点转换为PnP算法需要的格式
        std::vector<cv::Point2d> pu(p, p + 4);
        // cv::Mat pu(4, 1, CV_32FC2, const_cast<cv::Point2f*>(p));

        // PnP求解：从2D图像点和3D模型点求解相机位姿
        cv::Mat rvec, tvec;  // 旋转向量和平移向量
        bool success = cv::solvePnP(pw_cur, pu, F_MAT, C_MAT, rvec, tvec, false, cv::SOLVEPNP_IPPE);

        if (!success) {
            return false;
        }
        
        // === 坐标变换过程 ===
        // 获取装甲板中心在不同坐标系中的位置
        Pos3D pc, pw, pi;  // 相机坐标系、世界坐标系、IMU坐标系
        
        cv::cv2eigen(tvec, pc);  // PnP得到的是相机坐标系中的位置

        // std::cout << "T_camera2imu: " << T_camera2imu << std::endl;
        // std::cout << "R_camera2imu: " << R_camera2imu << std::endl;
        // std::cout << "R_world2imu: " << R_world2imu << std::endl;
        
        // 相机坐标系 -> IMU坐标系
        pi = R_camera2imu * pc + T_camera2imu;

        // IMU坐标系 -> 世界坐标系
        pw = R_world2imu.transpose() * pi;

        // === 计算装甲板四个角点的世界坐标 ===
        cv::Mat mat_R;
        Eigen::Matrix3d R;
        cv::Rodrigues(rvec, mat_R);  // 旋转向量转旋转矩阵
        cv::cv2eigen(mat_R, R);

        Pos3D p_a_w[4];  // 装甲板四角点世界坐标
        Pos3D p_a_c[4];  // 装甲板四角点相机坐标

        // 计算每个角点的3D坐标
        for (int i = 0; i < 4; ++i)
        {
            Pos3D temp(pw_cur[i].x, pw_cur[i].y, pw_cur[i].z);  // 装甲板模型坐标
            p_a_c[i] = R * temp + pc;  // 变换到相机坐标系
        }
        
        // === 计算装甲板法向量和朝向 ===
        // 计算装甲板在相机坐标系中的方向向量
        Pos3D armor_v_x_c, armor_v_y_c, armor_v_z_c;
        armor_v_x_c = (p_a_c[0] - p_a_c[3]) + (p_a_c[1] - p_a_c[2]);
        armor_v_y_c = (p_a_c[1] - p_a_c[0]) + (p_a_c[2] - p_a_c[3]);
        armor_v_z_c = armor_v_x_c.cross(armor_v_y_c);

        // 计算装甲板在相机坐标系中的偏航角
        // 偏航轴是负Z轴，因此使用-armor_v_z_c[2]
        // 正对装甲板的yaw_in_camera=0，绕世界坐标系z轴正方向旋转为负，相反为正
        yaw_in_camera = atan2(armor_v_z_c[0], -armor_v_z_c[2]);

        if (adjust)
            std::cout << pc[1] << std::endl << pc[0] << std::endl << pc[2] << std::endl << yaw_in_camera << std::endl;

        // === 组装最终测量值 ===
        // 测量值格式：[y坐标, x坐标, z坐标, 绝对偏航角]，该测量值对应于ekf的测量项
        // 绝对偏航角 = 相机坐标系中的偏航角 - 机器人姿态偏航角， [-pi, pi]
        measurement << pw[1], pw[0], pw[2], mathutils::limit_rad(yaw_in_camera - attitude_yaw);
        // measurement << pw[1], pw[0], pw[2], yaw_in_camera - attitude_yaw;

        return true;
    }
}

