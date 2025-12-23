#ifndef _ONNXMODULE_HPP_
#define _ONNXMODULE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "common.hpp" // 必须包含这个，因为 bbox_t 和 DetectionSource 在这里定义
#include "../backend.hpp"
#include <migraphx/migraphx.hpp>
#include <vector>

class ONNX : public BackEnd
{
public:
    static constexpr int INPUT_W = 640;
    static constexpr int INPUT_H = 640;
    static constexpr int TOPK_NUM = 128;
    static constexpr float CONF_THRESH = 0.65f; 
    // -ln(1/0.65 - 1) ≈ 0.619
    static constexpr float LOGIT_THRESH = 0.619f; 
    static constexpr float NMS_THRESH = 0.45f;

    explicit ONNX(const std::string &onnx_file);

    void build_engine_from_onnx(const std::string &onnx_file);
    void build_engine_from_cache(const std::string &cache_file_path);
    void cache_engine(const std::string &cache_file_path);

    ~ONNX();

    // 禁用拷贝
    ONNX(const ONNX &) = delete;
    ONNX operator=(const ONNX &) = delete;

    void operator()(const cv::Mat &src, std::vector<bbox_t> &det) override;

private:
    migraphx::program net;
    std::vector<float> inputTensorValues;
    
    // 用于后处理坐标还原的参数
    struct PreProcessParams {
        float scale;
        float ox;
        float oy;
    };
};

#endif /* _ONNXMODULE_HPP_ */