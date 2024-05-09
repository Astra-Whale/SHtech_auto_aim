//
// Inherit from SJTU-CV-2021/autoaim/detector/TRTModule.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <opencv2/core.hpp>
#include <onnxruntime_cxx_api.h>
#include "common.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>
#include <stdexcept>

/*
 * 四点模型
 */
class TRTModule
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.001f;

public:
    explicit TRTModule(const std::string &onnx_file);

    ~TRTModule();

    TRTModule(const TRTModule &) = delete;

    TRTModule operator=(const TRTModule &) = delete;

    //std::vector<bbox_t> operator()(const cv::Mat &src) const;
    void operator()(const cv::Mat &src, std::vector<bbox_t> &det);

private:
    Ort::Env env;
    Ort::SessionOptions sessionOptions;
    Ort::Session *session_ptr;
    Ort::AllocatorWithDefaultOptions allocator;
    int input_idx, output_idx;
    size_t input_sz, output_sz;
    std::vector<float> inputTensorValues;
    std::vector<float> outputTensorValues;

    std::vector<int64_t> inputDims;
    std::vector<int64_t> outputDims;

    std::vector<Ort::Value> inputTensors;
    std::vector<Ort::Value> outputTensors;
};

#endif /* _ONNXTRTMODULE_HPP_ */
