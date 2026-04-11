//
// Inherit from SJTU-CV-2021/autoaim/detector/AXCL.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#ifndef _AXCLMODULE_HPP_
#define _AXCLMODULE_HPP_

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

#include "ax_sys_api.h"
#include "ax_engine_api.h"

/*
 * 四点模型
 */
class AXCL:public BackEnd
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;

public:
    explicit AXCL(const std::string &AXCL_file);

    ~AXCL();

    AXCL(const AXCL &) = delete;

    AXCL operator=(const AXCL &) = delete;

    void operator()(const cv::Mat &src, std::vector<bbox_t> &det);

private:
    AX_ENGINE_HANDLE handle{};
    AX_ENGINE_IO_T io_data{};
    std::vector<char> model_buffer;
    std::vector<uint8_t> inputTensorValues;
    bool sys_initialized{false};
    bool engine_initialized{false};
    bool ready{false};

    void releaseIo();
    void releaseHandle();
    void shutdownRuntime();
};

#endif /* _AXCLMODULE_HPP_ */
