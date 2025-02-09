#ifndef _DETECT_BACKEND_HPP_
#define _DETECT_BACKEND_HPP_
#include <opencv2/core.hpp>
#include "common.hpp"
#include <string>
#include <vector>

class BackEnd
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;
public:
    BackEnd() = default;
    virtual ~BackEnd() = default;

    BackEnd(const BackEnd &) = delete;

    BackEnd operator=(const BackEnd &) = delete;

    virtual std::vector<bbox_t> operator()(const cv::Mat &src) const = 0;
    virtual void operator()(const cv::Mat &src, std::vector<bbox_t> &det) const = 0;
};

#endif //_DETECT_BACKEND_HPP_