//
// Inherit from SJTU-CV-2021/autoaim/detector/TRTModule.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <opencv2/core.hpp>
#include <NvInfer.h>
#include "common.hpp"

class Logger : public nvinfer1::ILogger
{
public:
    Logger() : ferr("error.txt"), finfo("info.txt"), fver("verbose.txt"), fwarn("warning.txt") {}
    void log(Severity severity, const char *msg) throw() override
    {
        //suppress info-level message
        switch (severity)
        {
        case Severity::kINTERNAL_ERROR:
        case Severity::kERROR:
            ferr << msg << std::endl;
        case Severity::kWARNING:
            fwarn << msg << std::endl;
        case Severity::kINFO:
            finfo << msg << std::endl;
        case Severity::kVERBOSE:
            fver << msg << std::endl;
        }
        if (severity == Severity::kERROR || severity == Severity::kWARNING)
            std::cout << "Log: " << msg << std::endl;
    }
    void StageLog(std::string msg)
    {
        ferr << msg << std::endl;
        fwarn << msg << std::endl;
        finfo << msg << std::endl;
        fver << msg << std::endl;
        std::cout << msg << std::endl;
    }
    std::ofstream ferr, finfo, fver, fwarn;
};

/*
 * 四点模型
 */
class TRTModule
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;

public:
    explicit TRTModule(const std::string &onnx_file);

    ~TRTModule();

    TRTModule(const TRTModule &) = delete;

    TRTModule operator=(const TRTModule &) = delete;

    std::vector<bbox_t> operator()(const cv::Mat &src) const;
    void operator()(const cv::Mat &src, std::vector<bbox_t> &det) const;

private:
    void build_engine_from_onnx(const std::string &onnx_file);

    void build_engine_from_cache(const std::string &cache_file);

    void cache_engine(const std::string &cache_file);

    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    mutable void *device_buffer[2];
    float *output_buffer;
    cudaStream_t stream;
    int input_idx, output_idx;
    size_t input_sz, output_sz;
};

#endif /* _ONNXTRTMODULE_HPP_ */
