//
// Inherit from SJTU-CV-2021/autoaim/detector/ONNX.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#ifndef _ONNXMODULE_HPP_
#define _ONNXMODULE_HPP_

#include <opencv2/core.hpp>
#include "common.hpp"
#include "../backend.hpp"
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

#include <migraphx/migraphx.hpp>
/*
 * 四点模型
 */
class ONNX:public BackEnd
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;

public:
    explicit ONNX(const std::string &onnx_file);

    void build_engine_from_onnx(const std::string &onnx_file);

    void build_engine_from_cache(const std::string &cache_file_path);

    void cache_engine(const std::string &cache_file_path);

    ~ONNX();

    ONNX(const ONNX &) = delete;

    ONNX operator=(const ONNX &) = delete;

    //std::vector<bbox_t> operator()(const cv::Mat &src) const;
    void operator()(const cv::Mat &src, std::vector<bbox_t> &det);

private:
    migraphx::program net;
    std::vector<float> inputTensorValues;
};

#endif /* _ONNXMODULE_HPP_ */
