#ifndef _ONNXMODULE_HPP_
#define _ONNXMODULE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "common.hpp" 
#include "../backend.hpp"
#include <migraphx/migraphx.hpp>
#include <vector>

class ONNX : public BackEnd
{
public:
    static constexpr int INPUT_W = 640;
    static constexpr int INPUT_H = 512;
    static constexpr int TOPK_NUM = 128;
    static constexpr float CONF_THRESH = 0.65f; 
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
};

#endif /* _ONNXMODULE_HPP_ */