// Copyright (C) 2025
// Licensed under the MIT License.

#ifndef ARMOR_CORNER_OPTIMIZER_HPP
#define ARMOR_CORNER_OPTIMIZER_HPP

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <array>
#include <optional>
#include <vector>
#include <cmath>
#include <iostream>

#include "common.hpp"

namespace detect
{
  struct CornerRefineCallStats
  {
    double roi_ms = 0.0;
    double preprocess_ms = 0.0;
    double find_light_ms = 0.0;
    double select_ms = 0.0;
    double final_check_ms = 0.0;
    double visualize_ms = 0.0;

    int candidate_light_bars = 0;
    int successful_lights = 0;
  };

  // Light structure to represent a light bar
  struct LightBar
  {
    cv::Point2f top;    // Top point of light
    cv::Point2f bottom; // Bottom point of light
    cv::Point2f center; // Center point of light
    float length;       // Length of light
    float width;        // Width of light
    float tilt_angle;   // Tilt angle of light
    LightBar() = default;

    LightBar(const cv::Point2f &t, const cv::Point2f &b)
        : top(t), bottom(b)
    {
      center = (top + bottom) * 0.5f;
      length = cv::norm(top - bottom);
      width = 0.0;      // Will be set during optimization
      tilt_angle = 0.0; // Will be calculated
    }
  };

  // Parameters for light bar optimization
  struct LightParams
  {
    float min_ratio;      // Minimum width/length ratio
    float max_ratio;      // Maximum width/length ratio
    float max_angle;      // Maximum tilt angle
    float min_fill_ratio; // Minimum fill ratio

    LightParams(
        float min_r = 0.07f,
        float max_r = 0.6f,
        float max_a = 30.0f,
        float min_fill = 0.5f)
        : min_ratio(min_r), max_ratio(max_r), max_angle(max_a), min_fill_ratio(min_fill) {}
  };

  // Parameters of the yolo model behaviors
  struct YoloModelCharacteristics
  {
      // 图像和装甲板基本参数
      int image_width;             // 图像宽度
      int image_height;            // 图像高度
      int min_light_height;        // 最小灯条高度
      int max_light_height;        // 最大灯条高度
      
      // YOLO模型行为特性
      float position_error_ratio;  // 位置偏离的比例(相对于灯条高度)
      float length_error_ratio;    // 长度测量误差比例
      float width_error_ratio;     // 宽度测量误差比例
      
      // ROI生成参数
      float roi_height_multiplier; // ROI高度相对于灯条高度的倍数
      float roi_width_multiplier;  // ROI宽度相对于灯条宽度的倍数
      int min_roi_size;            // 最小ROI尺寸
      
      // 灯条选择参数
      float center_distance_weight;    // 中心距离在选择算法中的权重
      float length_difference_weight;  // 长度差异在选择算法中的权重
      float angle_difference_weight;   // 角度差异在选择算法中的权重
      
      // 默认构造函数，设置默认值
      YoloModelCharacteristics() :
          image_width(1280),
          image_height(1024),
          min_light_height(10),
          max_light_height(120),
          position_error_ratio(0.5f),   // 位置可能偏离半个高度
          length_error_ratio(0.1f),     // 长度测量误差10%
          width_error_ratio(0.2f),      // 宽度测量误差20%
          // on axcl
          roi_height_multiplier(0.5f),  // ROI高度为灯条高度的1.3倍
          roi_width_multiplier(0.5f),   // ROI宽度为灯条宽度的1倍
          min_roi_size(20),             // 最小ROI尺寸为20像素
          center_distance_weight(0.2f), // 中心距离权重
          length_difference_weight(0.7f), // 长度差异权重
          angle_difference_weight(0.1f)   // 角度差异权重
      {}
  };

  // Class for optimizing armor corners using traditional CV methods
  class ArmorCornerOptimizer
  {
  public:
    // Constructor
    ArmorCornerOptimizer(
        const bool adjust_ = false,
        const int &bin_thres = 104,
        const LightParams &light_params = LightParams(),
        const YoloModelCharacteristics &yolo_params = YoloModelCharacteristics()
        );

    // Destructor
    ~ArmorCornerOptimizer() = default;

    /**
     * @brief Main function to optimize corners
     * @param input Original image
     * @param yolo_corners Four corners from YOLO (ordered as: left_top, left_bottom, right_top, right_bottom)
     * @return Optimized four corners
     */
    std::optional<std::array<cv::Point2f, 4>> optimizeCorners(
        const cv::Mat &input,
        const cv::Point2f yolo_corners[],
        const bool _show,
        CornerRefineCallStats* stats = nullptr
      );

    // Set parameters
    void setBinaryThreshold(int threshold) { binary_thres = threshold; }
    void setLightParams(const LightParams &params) { light_params = params; }
    void setYoloModelCharacteristics(const YoloModelCharacteristics &params) { yolo_params = params; }

  private:
    // Preprocess image
    cv::Mat preprocessImage(const cv::Mat &rgb_img, const cv::Rect &roi);

    // Find light bars in ROI
    std::vector<LightBar> findLightBars(const cv::Mat &binary_img, const cv::Rect &roi);

    // Check if a contour is a valid light bar
    bool isValidLightBar(const LightBar &light);

    cv::Rect calculateLightRoi(
        const cv::Point2f &center, 
        float roi_height,
        float roi_width,
        float light_length);

    cv::Rect validateRect(const cv::Rect &rect, int img_width, int img_height);

    int selectBestLightBar(
        const std::vector<LightBar> &light_bars, 
        const cv::Point2f &yolo_center,
        float expected_height,
        float expected_angle);
    
    cv::Mat visualizeROIs(const cv::Mat &input, const cv::Rect &left_roi, const cv::Rect &right_roi);

    cv::Mat visualizeBinaryResults(const cv::Mat &input, const cv::Mat &left_binary, const cv::Mat &right_binary, 
                                 const cv::Rect &left_roi, const cv::Rect &right_roi);

    // Parameters
    int binary_thres;
    LightParams light_params;
    YoloModelCharacteristics yolo_params;
  };

} // namespace detect

#endif // ARMOR_CORNER_OPTIMIZER_HPP
