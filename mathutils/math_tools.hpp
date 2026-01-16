/**
 * @file math_tools.hpp
 * @brief 数学工具模块 - 为自瞄预测系统提供数学计算函数
 * @author Cao Jingyan
 * @date 2025/11/16
 * 
 * 该模块提供：
 * 1. 弹道计算相关的数学函数
 * 2. 坐标变换和几何计算工具
 * 3. 目标预测所需的数值计算
 * 4. 装甲板几何分析函数
 */

#ifndef PREDICT_MATH_TOOLS_H
#define PREDICT_MATH_TOOLS_H

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

namespace mathutils
{
    /// @brief 三维位置向量类型别名，使用Eigen::Vector3d
    using Pos3D = Eigen::Vector3d;

    /// @brief 重力加速度常数 (m/s²)
    /// @note 根据具体地理位置调整的重力加速度值
    constexpr double g = 9.7946;

    /**
     * @brief 计算XY平面内的二维距离
     * @param m_pw 世界坐标系下的3D位置向量
     * @return 在XY平面内的距离（忽略Z分量）
     * @details 计算点到原点在水平面上的投影距离
     */
    static inline double distance_2D(const Pos3D &m_pw)
    {
        return sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0));
    }

    /**
     * @brief 计算三维欧几里得距离
     * @param m_pw 世界坐标系下的3D位置向量
     * @return 从原点到目标点的3D直线距离
     * @details 计算空间中两点间的直线距离
     */
    static inline double distance_3D(const Pos3D &m_pw)
    {
        return sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0) + m_pw(2, 0) * m_pw(2, 0));
    }

    /**
     * @brief 根据世界坐标计算俯仰角
     * @param m_pw 世界坐标系下的3D位置向量
     * @return 俯仰角（弧度制）- 仰角为负，俯角为正
     * @details 计算XY水平面与原点到目标连线的夹角
     */
    static inline double pw_to_pitch(const Pos3D &m_pw)
    {
        return std::atan2(m_pw(2, 0), distance_2D(m_pw));
    }

    /**
     * @brief 根据世界坐标计算偏航角
     * @param m_pw 世界坐标系下的3D位置向量
     * @return 偏航角（弧度制）- Y轴正方向为0, 绕z轴逆时针为正
     * @details 计算XY平面内从Y轴到目标方向的角度
     */
    static inline double pw_to_yaw(const Pos3D &m_pw)
    {
        return std::atan2(m_pw(1, 0), m_pw(0, 0));
    }

    /**
     * @brief 计算考虑重力影响的弹丸飞行时间
     * @param pw 目标在世界坐标系中的位置
     * @param shoot_speed 弹丸初始速度 (m/s)，默认23 m/s
     * @return 飞行时间（秒），如果目标不可达则返回-1
     * @details 使用弹道学轨迹方程求解最佳发射角和对应的飞行时间
     *          通过求解二次方程找到两种可能的弹道，选择较短时间的弹道
     *          考虑重力下降和初始速度的抛物线运动
     */
    static inline double cal_fly_time(const Pos3D &pw, double shoot_speed = 23.)
    {
        auto d = distance_2D(pw);  // 水平距离
        auto h = pw(2, 0);         // 高度差

        // 二次方程系数：at² + bt + c = 0
        auto a = g * d * d / (2 * shoot_speed * shoot_speed);
        auto b = -d;
        auto c = a + h;
        auto delta = b * b - 4 * a * c;  // 判别式

        if (delta < 0) {
            return -1;  // 无实数解 - 目标不可达
        }

        // 计算两种可能的发射角
        auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
        auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
        auto pitch_1 = std::atan(tan_pitch_1);
        auto pitch_2 = std::atan(tan_pitch_2);
        
        // 计算两种弹道的飞行时间
        auto t_1 = d / (shoot_speed * std::cos(pitch_1));
        auto t_2 = d / (shoot_speed * std::cos(pitch_2));

        // 返回较短的飞行时间（较平的弹道）
        auto fly_time = (t_1 < t_2) ? t_1 : t_2;

        return fly_time;
    }

    /**
     * @brief 计算四边形的几何中心
     * @param pts 四个顶点坐标的数组
     * @return 中心点坐标，为四个顶点坐标的平均值
     * @details 通过求四个角点坐标的算术平均值得到几何中心
     */
    static inline cv::Point2f points_center(cv::Point2f pts[4])
    {
        cv::Point2f center;
        center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;
        center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4;
        return center;
    }

    /**
     * @brief 将四边形转换为包围矩形
     * @param armor 装甲板检测边界框，包含4个角点坐标
     * @param coefficient ROI尺寸的缩放系数，默认1.0
     * @return 包围四边形的轴对齐矩形，可选择缩放
     * @details 从任意四边形创建轴对齐的包围矩形
     */
    static inline cv::Rect2f get_ROI(bbox_t &armor, float coefficient = 1.0f)
    {
        auto center = points_center(armor.pts);
        
        // 找到包围框的宽度和高度
        auto w = std::max({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x}) -
                 std::min({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x});
        auto h = std::max({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y}) -
                 std::min({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y});
        
        return cv::Rect2f(center.x - w / 2, center.y - h / 2, w * coefficient, h * coefficient);
    }

    /**
     * @brief 使用布雷齐施耐德公式计算四边形面积
     * @param bx 包含4个角点的边界框
     * @return 四边形的面积（像素²）
     * @details 使用布雷齐施耐德公式计算一般四边形面积
     *          适用于凸四边形的面积计算，用于装甲板大小评估
     */
    static inline double get_bbox_size(const bbox_t &bx)
    {
        // 计算四条边长
        auto bx_a = sqrt(pow(bx.pts[0].x - bx.pts[1].x, 2) + pow(bx.pts[0].y - bx.pts[1].y, 2));
        auto bx_b = sqrt(pow(bx.pts[1].x - bx.pts[2].x, 2) + pow(bx.pts[1].y - bx.pts[2].y, 2));
        auto bx_c = sqrt(pow(bx.pts[2].x - bx.pts[3].x, 2) + pow(bx.pts[2].y - bx.pts[3].y, 2));
        auto bx_d = sqrt(pow(bx.pts[3].x - bx.pts[0].x, 2) + pow(bx.pts[3].y - bx.pts[0].y, 2));
        
        // 布雷齐施耐德公式的半周长
        auto bx_z = (bx_a + bx_b + bx_c + bx_d) / 2;
        
        // 使用布雷齐施耐德公式计算面积（假设为凸四边形）
        auto bx_size = 2 * sqrt((bx_z - bx_a) * (bx_z - bx_b) * (bx_z - bx_c) * (bx_z - bx_d));
        return bx_size;
    }
}

#endif // PREDICT_MATH_TOOLS_H
