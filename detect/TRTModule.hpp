//
// Inherit from SJTU-CV-2021/autoaim/detector/TRTModule.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <opencv2/core.hpp>
#include "ncnn/mat.h"
#include "ncnn/net.h"
#include "common.hpp"

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
    void load_engine(const std::string &onnx_file);
    ncnn::Net net;
    ncnn::Extractor* ex;
    int input_idx, output_idx;
    size_t input_sz, output_sz;
};

#endif /* _ONNXTRTMODULE_HPP_ */
