//
// Modified for New RoboMaster Armor Detection Model (YOLO-Pose style)
// Supports: Letterbox, Fail-Fast Logit Filter, Inverse Affine Restore
//

#ifndef _ONNXMODULE_HPP_
#define _ONNXMODULE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> // for warpAffine
#include "common.hpp"
#include "../backend.hpp"
#include <migraphx/migraphx.hpp>
#include <vector>
#include <cmath>

class ONNX : public BackEnd
{
public:
    // 配置参数
    static constexpr int INPUT_W = 640;
    static constexpr int INPUT_H = 640;
    static constexpr int TOPK_NUM = 128;
    
    // 置信度阈值 (Confidence Threshold)
    static constexpr float CONF_THRESH = 0.65f; 
    
    // Fail-Fast Logit 阈值: -ln(1/CONF_THRESH - 1)
    // 当 conf = 0.65 时, logit ≈ 0.619
    // 只有 logit > LOGIT_THRESH，Sigmoid后的值才会 > CONF_THRESH
    static constexpr float LOGIT_THRESH = 0.619f; 
    
    static constexpr float NMS_THRESH = 0.45f;

    explicit ONNX(const std::string &onnx_file);

    void build_engine_from_onnx(const std::string &onnx_file);
    void build_engine_from_cache(const std::string &cache_file_path);
    void cache_engine(const std::string &cache_file_path);

    ~ONNX();

    ONNX(const ONNX &) = delete;
    ONNX operator=(const ONNX &) = delete;

    void operator()(const cv::Mat &src, std::vector<bbox_t> &det) override;

private:
    migraphx::program net;
    std::vector<float> inputTensorValues;
    
    // 用于保存预处理的缩放参数，供后处理还原坐标使用
    struct PreProcessParams {
        float scale;
        float ox;
        float oy;
    };
};

#endif /* _ONNXMODULE_HPP_ */