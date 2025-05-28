// Copyright (C) 2025
// Licensed under the MIT License.

#include "armor_corner_optimizer.hpp"

namespace detect
{

  ArmorCornerOptimizer::ArmorCornerOptimizer(const int &bin_thres, const LightParams &light_params,const YoloModelCharacteristics &yolo_params)
  : binary_thres(bin_thres), light_params(light_params), yolo_params(yolo_params)
  {
  }
  
    std::vector<cv::Point2f> ArmorCornerOptimizer::optimizeCorners(
      const cv::Mat &input,
      const cv::Point2f yolo_corners[],
      bool _show)
  {

      std::vector<cv::Point2f> optimized_corners;

      for(int i=0;i<4;i++){
        optimized_corners.push_back(yolo_corners[i]);
      }

      // Create debug image if needed
      debug_img = input.clone();

      // Calculate centers, estimated heights and angles of light bars from YOLO corners
      cv::Point2f left_center = (yolo_corners[0] + yolo_corners[1]) * 0.5f;
      cv::Point2f right_center = (yolo_corners[2] + yolo_corners[3]) * 0.5f;
      
      cv::Vec2f left_vector = yolo_corners[1] - yolo_corners[0];
      float left_height = std::abs(left_vector[1]); // Height as y-component
      float left_width = std::abs(left_vector[0]); // Width as x-component
      float left_length = cv::norm(left_vector); // Length as vector norm

      cv::Vec2f right_vector = yolo_corners[2] - yolo_corners[3];
      float right_height = std::abs(right_vector[1]); // Height as y-component
      float right_width = std::abs(right_vector[0]); // Width as x-component
      float right_length = cv::norm(right_vector); // Length as vector norm
      
      // 计算left_vector与Y轴的最小夹角
      float left_angle_rad = std::atan2(left_vector[0], left_vector[1]); // 注意这里是(x, y)
      float left_angle_deg = std::abs(left_angle_rad * 180.0f / CV_PI);

      if (left_angle_deg > 90.0f) {
          left_angle_deg = 180.0f - left_angle_deg;
      }

      // 计算right_vector与Y轴的最小夹角
      float right_angle_rad = std::atan2(right_vector[0], right_vector[1]);
      float right_angle_deg = std::abs(right_angle_rad * 180.0f / CV_PI);

      if (right_angle_deg > 90.0f) {
          right_angle_deg = 180.0f - right_angle_deg;
      }

      // Validate light bar heights are within expected range
      left_length = std::max(float(yolo_params.min_light_height), 
                            std::min(left_height, float(yolo_params.max_light_height)));
      right_length = std::max(float(yolo_params.min_light_height), 
                            std::min(right_height, float(yolo_params.max_light_height)));
      
      // Calculate ROIs with adaptive parameters
      cv::Rect left_roi = calculateLightRoi(left_center, left_height, left_width,left_length);
      cv::Rect right_roi = calculateLightRoi(right_center, right_height, right_width,right_length);

    //       // 输出左 ROI 参数及结果
    // std::cout << "Left ROI Input Parameters:" << std::endl;
    // std::cout << "  center: (" << left_center.x << ", " << left_center.y << ")" << std::endl;
    // std::cout << "  height: " << left_height << std::endl;
    // std::cout << "  width: " << left_width << std::endl;
    // std::cout << "  length: " << left_length << std::endl;
    // std::cout << "Left ROI Result: x=" << left_roi.x << ", y=" << left_roi.y 
    //           << ", width=" << left_roi.width << ", height=" << left_roi.height << std::endl;

    // // 输出右 ROI 参数及结果
    // std::cout << "\nRight ROI Input Parameters:" << std::endl;
    // std::cout << "  center: (" << right_center.x << ", " << right_center.y << ")" << std::endl;
    // std::cout << "  height: " << right_height << std::endl;
    // std::cout << "  width: " << right_width << std::endl;
    // std::cout << "  length: " << right_length << std::endl;
    // std::cout << "Right ROI Result: x=" << right_roi.x << ", y=" << right_roi.y 
    //           << ", width=" << right_roi.width << ", height=" << right_roi.height << std::endl;
      
      // Ensure ROIs stay within image boundaries
      left_roi = validateRect(left_roi, input.cols, input.rows);
      right_roi = validateRect(right_roi, input.cols, input.rows);

      if (left_roi.x < 0 || left_roi.y < 0 || left_roi.width < 0 || left_roi.height < 0 || right_roi.x < 0 || right_roi.y < 0 || right_roi.width < 0 || right_roi.height < 0) {
        optimized_corners.clear();
        return optimized_corners;
      }

      // Rest of the processing remains the same
      cv::Mat left_binary = preprocessImage(input, left_roi,left_length);
      cv::Mat right_binary = preprocessImage(input, right_roi,right_length);

      std::vector<LightBar> left_lights = findLightBars(input, left_binary, left_roi);
      std::vector<LightBar> right_lights = findLightBars(input, right_binary, right_roi);

      // If light bars are found, update corners
      if (!left_lights.empty())
      {
          // Find best matching light bar
          int best_idx = selectBestLightBar(left_lights, left_center, left_height, left_angle_deg);
          if(best_idx!=-1)
          {
          // Update optimized corners with the found light bar
          optimized_corners[0] = left_lights[best_idx].top;
          optimized_corners[1] = left_lights[best_idx].bottom;
          }
      }

      if (!right_lights.empty())
      {
          // Find best matching light bar
          int best_idx = selectBestLightBar(right_lights, right_center, right_height, right_angle_deg);
          if(best_idx!=-1)
          {
          // Update optimized corners with the found light bar
          optimized_corners[3] = right_lights[best_idx].top;
          optimized_corners[2] = right_lights[best_idx].bottom;
          }
      }

      // create visulaztion for conor optimizer
      if(_show)
      {
        cv::Mat roi_visualization = visualizeROIs(input, left_roi, right_roi);
        cv::Mat binary_visualization = visualizeBinaryResults(input, left_binary, right_binary, left_roi, right_roi);
        cv::imshow("ROI Visualization", roi_visualization);
        cv::imshow("Binary Visualization", binary_visualization);
      }

      return optimized_corners;
  }

  cv::Rect ArmorCornerOptimizer::calculateLightRoi(
      const cv::Point2f &center, 
      float roi_height,
      float roi_width,
      float light_length
    )
  {
      roi_height += light_length * yolo_params.roi_height_multiplier;
      roi_width += light_length * yolo_params.roi_width_multiplier;
      // Create ROI with adaptive size
      cv::Rect roi(
          int(center.x - roi_width * 0.5f),
          int(center.y - roi_height * 0.5f),
          int(roi_width),
          int(roi_height)
      );
      
      // Ensure minimum ROI size
      roi.width = std::max(yolo_params.min_roi_size, roi.width);
      roi.height = std::max(yolo_params.min_roi_size, roi.height);
      
      return roi;
  }

  cv::Rect ArmorCornerOptimizer::validateRect(const cv::Rect &rect, int img_width, int img_height)
  {
      cv::Rect valid_rect = rect;
      
      // Ensure x, y are non-negative
      valid_rect.x = std::max(0, valid_rect.x);
      valid_rect.y = std::max(0, valid_rect.y);
      
      // Ensure width and height don't exceed image dimensions
      valid_rect.width = std::min(img_width - valid_rect.x, valid_rect.width);
      valid_rect.height = std::min(img_height - valid_rect.y, valid_rect.height);
      
      return valid_rect;
  }

  int ArmorCornerOptimizer::selectBestLightBar(
      const std::vector<LightBar> &light_bars, 
      const cv::Point2f &yolo_center,
      float expected_height,
      float expected_angle)
  {
      int best_idx = 0;
      float best_score = std::numeric_limits<float>::max();
      
      for (size_t i = 0; i < light_bars.size(); ++i)
      {
          // Calculate distance between centers (normalized by expected height)
          float center_dist = cv::norm(light_bars[i].center - yolo_center) / expected_height;
          
          // Calculate height difference ratio
          float height_diff = std::abs(light_bars[i].length - expected_height) / expected_height;
          
          // Calculate angle difference (normalized to 0-1 range, assuming max 90 degree difference)
          float angle_diff = std::abs(light_bars[i].tilt_angle - expected_angle) / 90.0f;
          
          // Combined weighted score (lower is better)
          float score = 
              center_dist * yolo_params.center_distance_weight +
              height_diff * yolo_params.length_difference_weight +
              angle_diff * yolo_params.angle_difference_weight;
          
          if (score < best_score)
          {
              best_score = score;
              best_idx = i;
          }
      }

      if(best_score>0.3)
      {
        best_idx = -1;
      }

      
      return best_idx;
  }
    
  cv::Mat ArmorCornerOptimizer::preprocessImage(const cv::Mat &rgb_img, const cv::Rect &roi, float light_length)
  {
    std::cout << roi << std::endl; 

    // Extract ROI from the image
    cv::Mat roi_img = rgb_img(roi);
    
    // Fast green channel extraction
    cv::Mat green_channel(roi_img.rows, roi_img.cols, CV_8UC1);
    for (int i = 0; i < roi_img.rows; i++) {
      const uchar* src = roi_img.ptr<uchar>(i);
      uchar* dst = green_channel.ptr<uchar>(i);
      for (int j = 0; j < roi_img.cols; j++) {
        *dst++ = src[1]; // Green channel in BGR
        src += 3;        // Move to next pixel
      }
    }
    
    // cv::Mat green_channel;
    // cv::cvtColor(roi_img, green_channel, cv::COLOR_RGB2GRAY);

    // Calculate the light bar area: length × (length/4) considering 4:1 ratio
    float light_area = light_length * (light_length / 4.0f);
    
    // Calculate fraction of pixels that should be bright
    float bright_pixel_ratio = light_area / (roi.width * roi.height);

    bright_pixel_ratio *= 0.6;
    
    // Add tolerance and clamp to reasonable range
    bright_pixel_ratio = std::min(0.3f, std::max(0.05f, bright_pixel_ratio * 1.2f));
    
    // Fast histogram calculation
    int histogram[256] = {0};
    for (int i = 0; i < green_channel.rows; i++) {
      const uchar* row = green_channel.ptr<uchar>(i);
      for (int j = 0; j < green_channel.cols; j++) {
        histogram[row[j]]++;
      }
    }
    
    // Determine threshold to keep exactly the calculated ratio of bright pixels
    int total_pixels = green_channel.rows * green_channel.cols;
    int pixels_to_keep = static_cast<int>(total_pixels * bright_pixel_ratio);
    int cumulative_count = 0;
    int threshold = 0;
    
    for (int i = 255; i >= 0; i--) {
      cumulative_count += histogram[i];
      if (cumulative_count >= pixels_to_keep) {
        threshold = i;
        break;
      }
    }

    threshold = 150;
    // std::cout<<"thre"<<threshold<<std::endl;
    
    // Apply threshold
    cv::Mat binary_img;
    cv::threshold(green_channel, binary_img, threshold, 255, cv::THRESH_BINARY);
    
    // Morphological operation for noise removal
    if (roi.width > 10 && roi.height > 10) {
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
      cv::morphologyEx(binary_img, binary_img, cv::MORPH_CLOSE, element);
    }
    
    return binary_img;
  }

  std::vector<LightBar> ArmorCornerOptimizer::findLightBars(
      const cv::Mat &rgb_img, const cv::Mat &binary_img, const cv::Rect &roi)
  {
    // Find contours in binary image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> light_bars;

    // Process each contour
    for (const auto &contour : contours)
    {
      // Skip small contours
      if (contour.size() < 5)
        continue;

      // Get bounding rectangle and rotated rectangle
      auto b_rect = cv::boundingRect(contour);
      auto r_rect = cv::minAreaRect(contour);

      // Create mask for the contour
      cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
      std::vector<cv::Point> mask_contour;
      for (const auto &p : contour)
      {
        mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
      }
      cv::fillPoly(mask, {mask_contour}, 255);

      // Find non-zero points in mask
      std::vector<cv::Point> points;
      cv::findNonZero(mask, points);

      // Check fill ratio
      bool is_fill_rotated_rect =
          points.size() / (r_rect.size.width * r_rect.size.height) > light_params.min_fill_ratio;

      if (!is_fill_rotated_rect)
        continue;

      // Fit line to points
      cv::Vec4f line_params;
      cv::fitLine(points, line_params, cv::DIST_L2, 0, 0.01, 0.01);

      // Calculate top and bottom points
      cv::Point2f top, bottom;
      float angle_k;

      if (int(line_params[0] * 100) == 100 || int(line_params[1] * 100) == 0)
      {
        // Vertical line
        top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y) + cv::Point2f(roi.x, roi.y);
        bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height) + cv::Point2f(roi.x, roi.y);
        angle_k = 0;
      }
      else
      {
        // Angled line
        float k = line_params[1] / line_params[0];
        float b = (line_params[3] + b_rect.y) - k * (line_params[2] + b_rect.x);

        // Calculate line endpoints
        top = cv::Point2f((b_rect.y - b) / k, b_rect.y) + cv::Point2f(roi.x, roi.y);
        bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height) + cv::Point2f(roi.x, roi.y);

        // Calculate angle
        angle_k = std::abs(std::atan(k) / CV_PI * 180 - 90);
        if (angle_k > 90)
        {
          angle_k = 180 - angle_k;
        }
      }

      // Create light bar
      LightBar light(top, bottom);
      light.width = cv::norm(cv::Point2f(r_rect.size.width, r_rect.size.height)) / 2.0;
      light.tilt_angle = angle_k;

      // Determine color if needed
      if (b_rect.x >= 0 && b_rect.y >= 0 &&
          b_rect.x + b_rect.width <= rgb_img.cols &&
          b_rect.y + b_rect.height <= rgb_img.rows)
      {

        cv::Rect adjusted_rect(b_rect.x + roi.x, b_rect.y + roi.y, b_rect.width, b_rect.height);

        if (adjusted_rect.x >= 0 && adjusted_rect.y >= 0 &&
            adjusted_rect.x + adjusted_rect.width <= rgb_img.cols &&
            adjusted_rect.y + adjusted_rect.height <= rgb_img.rows)
        {

          auto roi_img = rgb_img(adjusted_rect);
          int sum_r = 0, sum_b = 0;

          // Iterate through the ROI to determine color
          for (int i = 0; i < roi_img.rows; i++)
          {
            for (int j = 0; j < roi_img.cols; j++)
            {
              cv::Point global_pt(j + adjusted_rect.x, i + adjusted_rect.y);
              cv::Point local_pt(global_pt.x - roi.x, global_pt.y - roi.y);

              if (local_pt.x >= 0 && local_pt.y >= 0 &&
                  local_pt.x < binary_img.cols && local_pt.y < binary_img.rows)
              {
                if (binary_img.at<uchar>(local_pt) > 0)
                {
                  sum_r += roi_img.at<cv::Vec3b>(i, j)[0]; // R channel
                  sum_b += roi_img.at<cv::Vec3b>(i, j)[2]; // B channel
                }
              }
            }
          }

          light.color = sum_r > sum_b ? RED : BLUE;
        }
      }

      // Validate light bar
      if (isValidLightBar(light))
      {
        light_bars.push_back(light);

        // Draw on debug image
        cv::line(debug_img, light.top, light.bottom, cv::Scalar(0, 255, 0), 2);
      }
    }

    return light_bars;
  }

  bool ArmorCornerOptimizer::isValidLightBar(const LightBar &light)
  {
    // Calculate width/length ratio
    float ratio = light.width / light.length;
    bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;

    // Check angle
    bool angle_ok = light.tilt_angle < light_params.max_angle;

    return ratio_ok && angle_ok;
  }

  cv::Mat ArmorCornerOptimizer::visualizeROIs(const cv::Mat &input, const cv::Rect &left_roi, const cv::Rect &right_roi)
  {
    // 创建黑色背景图像（与输入图像相同大小）
    cv::Mat visualization = cv::Mat::zeros(input.size(), input.type());
    
    // 确保ROI在图像范围内
    cv::Rect valid_left_roi = validateRect(left_roi, input.cols, input.rows);
    cv::Rect valid_right_roi = validateRect(right_roi, input.cols, input.rows);
    
    // 将原始图像的ROI复制到可视化图像的对应位置
    if (valid_left_roi.width > 0 && valid_left_roi.height > 0) {
        input(valid_left_roi).copyTo(visualization(valid_left_roi));
    }
    
    if (valid_right_roi.width > 0 && valid_right_roi.height > 0) {
        input(valid_right_roi).copyTo(visualization(valid_right_roi));
    }
    
    // 绘制ROI边界框
    cv::rectangle(visualization, valid_left_roi, cv::Scalar(0, 255, 0), 2);
    cv::rectangle(visualization, valid_right_roi, cv::Scalar(0, 255, 0), 2);
    
    return visualization;
  }

  cv::Mat ArmorCornerOptimizer::visualizeBinaryResults(
    const cv::Mat &input, 
    const cv::Mat &left_binary, 
    const cv::Mat &right_binary,
    const cv::Rect &left_roi, 
    const cv::Rect &right_roi)
  {
    // 创建黑色背景图像（与输入图像相同大小）
    cv::Mat visualization = cv::Mat::zeros(input.size(), input.type());
    
    // 确保ROI在图像范围内
    cv::Rect valid_left_roi = validateRect(left_roi, input.cols, input.rows);
    cv::Rect valid_right_roi = validateRect(right_roi, input.cols, input.rows);
    
    // 将二值化结果转换为彩色图像（白色区域将显示为绿色）
    if (valid_left_roi.width > 0 && valid_left_roi.height > 0 && 
        left_binary.cols == valid_left_roi.width && left_binary.rows == valid_left_roi.height) {
        
        // 为了更好的可视化，将二值图像转换为彩色图像
        cv::Mat colored_binary = cv::Mat::zeros(left_binary.size(), CV_8UC3);
        for (int y = 0; y < left_binary.rows; ++y) {
            for (int x = 0; x < left_binary.cols; ++x) {
                if (left_binary.at<uchar>(y, x) > 0) {
                    colored_binary.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // 绿色表示二值化后的亮区
                }
            }
        }
        
        // 复制到可视化图像中
        colored_binary.copyTo(visualization(valid_left_roi));
    }
    
    if (valid_right_roi.width > 0 && valid_right_roi.height > 0 && 
        right_binary.cols == valid_right_roi.width && right_binary.rows == valid_right_roi.height) {
        
        // 为了更好的可视化，将二值图像转换为彩色图像
        cv::Mat colored_binary = cv::Mat::zeros(right_binary.size(), CV_8UC3);
        for (int y = 0; y < right_binary.rows; ++y) {
            for (int x = 0; x < right_binary.cols; ++x) {
                if (right_binary.at<uchar>(y, x) > 0) {
                    colored_binary.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // 绿色表示二值化后的亮区
                }
            }
        }
        
        // 复制到可视化图像中
        colored_binary.copyTo(visualization(valid_right_roi));
    }
    
    // 绘制ROI边界框
    cv::rectangle(visualization, valid_left_roi, cv::Scalar(0, 255, 255), 2);
    cv::rectangle(visualization, valid_right_roi, cv::Scalar(0, 255, 255), 2);
    
    return visualization;
  }

} // namespace detect