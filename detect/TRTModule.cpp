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

static inline bool is_overlap(const float pts1[8], const float pts2[8])
{
    cv::Rect2f bbox1, bbox2;
    bbox1.x = reduce_min(pts1[0], pts1[2], pts1[4], pts1[6]);
    bbox1.y = reduce_min(pts1[1], pts1[3], pts1[5], pts1[7]);
    bbox1.width = reduce_max(pts1[0], pts1[2], pts1[4], pts1[6]) - bbox1.x;
    bbox1.height = reduce_max(pts1[1], pts1[3], pts1[5], pts1[7]) - bbox1.y;
    bbox2.x = reduce_min(pts2[0], pts2[2], pts2[4], pts2[6]);
    bbox2.y = reduce_min(pts2[1], pts2[3], pts2[5], pts2[7]);
    bbox2.width = reduce_max(pts2[0], pts2[2], pts2[4], pts2[6]) - bbox2.x;
    bbox2.height = reduce_max(pts2[1], pts2[3], pts2[5], pts2[7]) - bbox2.y;
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
    net.load_param("/home/magician/Downloads/param");
    net.load_model("/home/magician/Downloads/model-opt-4-sim-opt.bin");
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
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(x.data, ncnn::Mat::PIXEL_RGB, x.cols, x.rows, x.cols, x.rows);
    ex.input("images", in);
    ncnn::Mat out;
    ex.extract("output", out);
    // post-process [nms]
    /*
    det.reserve(TOPK_NUM);
    std::vector<uint8_t> removed(TOPK_NUM);
    for (int i = 0; i < TOPK_NUM; i++)
    {
        auto *box_buffer = output_buffer + i * 20; // 20->23
        if (box_buffer[8] < inv_sigmoid(KEEP_THRES))
            break;
        if (removed[i])
            continue;
        det.emplace_back();
        auto &box = det.back();
        memcpy(&box.pts, box_buffer, 8 * sizeof(float));
        for (auto &pt : box.pts)
            pt.x *= fx, pt.y *= fy;
        box.confidence = sigmoid(box_buffer[8]);
        box.color_id = argmax(box_buffer + 9, 4);
        box.tag_id = argmax(box_buffer + 13, 7);
        for (int j = i + 1; j < TOPK_NUM; j++)
        {
            auto *box2_buffer = output_buffer + j * 20;
            if (box2_buffer[8] < inv_sigmoid(KEEP_THRES))
                break;
            if (removed[j])
                continue;
            if (is_overlap(box_buffer, box2_buffer))
                removed[j] = true;
        }
    }*/
}
