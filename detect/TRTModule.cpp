//
// Inherit from SJTU-CV-2021/autoaim/detector/TRTModule.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#include "TRTModule.hpp"
#include <fstream>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>


template <class F, class T, class... Ts>
T reduce(F &&func, T x, Ts... xs)
{
    if constexpr (sizeof...(Ts) > 0)
    {
        return func(x, reduce(std::forward<F>(func), xs...));
    }
    else
    {
        return x;
    }
}

template <class T, class... Ts>
T reduce_max(T x, Ts... xs)
{
    return reduce([](auto &&a, auto &&b)
                  { return std::max(a, b); },
                  x, xs...);
}

template <class T, class... Ts>
T reduce_min(T x, Ts... xs)
{
    return reduce([](auto &&a, auto &&b)
                  { return std::min(a, b); },
                  x, xs...);
}

static inline cv::Rect2f gen_rect_bbox(const bbox_t &box)
{
    cv::Rect2f rect_bbox;
    float* pts = (float*)box.pts;
    rect_bbox.x = reduce_min(pts[0], pts[2], pts[4], pts[6]);
    rect_bbox.y = reduce_min(pts[1], pts[3], pts[5], pts[7]);
    rect_bbox.width = reduce_max(pts[0], pts[2], pts[4], pts[6]) - rect_bbox.x;
    rect_bbox.height = reduce_max(pts[1], pts[3], pts[5], pts[7]) - rect_bbox.y;
    return rect_bbox;
}

static inline bool is_overlap(const bbox_t &box1, const bbox_t &box2)
{
    cv::Rect2f bbox1 = gen_rect_bbox(box1);
    cv::Rect2f bbox2 = gen_rect_bbox(box2);
    return (bbox1 & bbox2).area() > 0;
}

static inline int argmax(const float *ptr, int len)
{
    int max_arg = 0;
    for (int i = 1; i < len; i++)
    {
        if (ptr[i] > ptr[max_arg])
            max_arg = i;
    }
    return max_arg;
}

constexpr float inv_sigmoid(float x)
{
    return -std::log(1 / x - 1);
}

constexpr float sigmoid(float x)
{
    return 1 / (1 + std::exp(-x));
}

TRTModule::TRTModule(const std::string &onnx_file)
{
    load_engine(onnx_file);
}

TRTModule::~TRTModule()
{
}

void TRTModule::load_engine(const std::string &onnx_file)
{
    net.opt.use_vulkan_compute = true;
    net.load_param("asset/ncnn.param");
    net.load_model("asset/ncnn.bin");
}

void TRTModule::operator()(const cv::Mat &src, std::vector<bbox_t> &det)
{
    // pre-process [bgr2rgb & resize]
    det.clear();
    cv::Mat x;
    float fx = (float)src.cols / 640.f, fy = (float)src.rows / 384.f;
    //cv::cvtColor(src, x, cv::COLOR_BGR2RGB);
    x = src;
    if (src.cols != 640 || src.rows != 384)
    {
        cv::resize(x, x, {640, 384});
    }
    
    ncnn::Extractor ex = net.create_extractor();
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(x.data, ncnn::Mat::PIXEL_RGB, x.cols, x.rows, 640, 384);
    ex.input("images", in);
    ncnn::Mat out;
    auto ret = ex.extract("129", out);
    std::vector<bbox_t> candidates;
    for (size_t i = 0; i < out.h; i++)
    {
        const float* box_buffer = out.row(i);
        if (box_buffer[8] < inv_sigmoid(KEEP_THRES))
            continue;
        candidates.emplace_back();
        auto &box = candidates.back();
        memcpy(&box.pts, box_buffer, 8 * sizeof(float));
        for (auto &pt : box.pts)
            pt.x *= fx, pt.y *= fy;
        box.confidence = sigmoid(box_buffer[8]);
        box.color_id = argmax(box_buffer + 9, 4);
        box.tag_id = argmax(box_buffer + 13, 7);
    }
    std::sort(candidates.begin(), candidates.end(), std::greater<bbox_t>());
    // post-process [nms]
    det.reserve(TOPK_NUM);
    std::vector<uint8_t> removed(TOPK_NUM);
    for (int i = 0; i < TOPK_NUM && i < candidates.size(); i++)
    {
        if (removed[i])
            continue;
    	auto& box1 = candidates.at(i);
        for (int j = i + 1; j < TOPK_NUM && j < candidates.size(); j++)
        {
            auto& box2 = candidates.at(j);
            if (removed[j])
                continue;
            if (is_overlap(box1, box2))
                removed[j] = true;
        }
    }
    std::cout<<ret<<"ret/"<<out.w<<","<<out.h<<","<<out.c<<"candidates:"<<candidates.size()<<"final:"<<det.size()<<std::endl;
}
