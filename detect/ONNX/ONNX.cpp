//
// Inherit from SJTU-CV-2021/autoaim/detector/ONNX.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#include "ONNX.hpp"
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

ONNX::ONNX(const std::string &onnx_file) : BackEnd() 
{
    std::filesystem::path onnx_file_path(onnx_file);
    auto cache_file_path = onnx_file_path;
    cache_file_path.replace_extension("cache");
    if (std::filesystem::exists(cache_file_path))
    {
        build_engine_from_cache(cache_file_path.c_str());
    }
    else
    {
        build_engine_from_onnx(onnx_file_path.c_str());
        cache_engine(cache_file_path.c_str());
    }
}

void ONNX::build_engine_from_onnx(const std::string &onnx_file)
{
    migraphx::onnx_options onnx_opts;
    // import and parse onnx file into migraphx::program
    net = parse_onnx(onnx_file.c_str(), onnx_opts);
    // print imported model
    net.print();
    migraphx::target targ = migraphx::target("gpu");
    migraphx::compile_options comp_opts;
    // migraphx::quantize_int8_options qopts;
    migraphx::quantize_fp16(net);
    comp_opts.set_offload_copy();

    net.compile(targ, comp_opts);

    net.print();
}

void ONNX::build_engine_from_cache(const std::string &cache_file_path)
{
    net = migraphx::load(cache_file_path.c_str());
}

void ONNX::cache_engine(const std::string &cache_file_path)
{
    migraphx::save(net, cache_file_path.c_str());
}

ONNX::~ONNX()
{
}

void ONNX::operator()(const cv::Mat &src, std::vector<bbox_t> &det)
{
//     auto t1 = std::chrono::steady_clock::now();
    // pre-process [bgr2rgb & resize]
    det.clear();
    cv::Mat x;
    cv::Mat preprocessedImage;
    float fx = (float)src.cols / 640.f, fy = (float)src.rows / 384.f;
    // 废弃的预处理代码
    // cv::cvtColor(src, x, cv::COLOR_BGR2RGB);
    // if (src.cols != 640 || src.rows != 384)
    // {
    //     cv::resize(x, x, {640, 384});
    // }
    // x.convertTo(x, CV_32F, 1.0 / 255);

    // // step 8: Convert the image to CHW RGB float format.
    // // HWC to CHW
    // cv::dnn::blobFromImage(x, preprocessedImage);

    cv::dnn::blobFromImage(
        src, 
        preprocessedImage, 
        1.0 / 255.0,          // 归一化（等同于你的 convertTo）
        cv::Size(640, 384),   // 自动缩放
        cv::Scalar(),         // 不减均值
        false                  // swapRB: BGR→RGB（关键！）
    );


    inputTensorValues.assign(preprocessedImage.begin<float>(), preprocessedImage.end<float>());

    migraphx::program_parameters prog_params;
    auto param_shapes = net.get_parameter_shapes();
    auto input        = param_shapes.names().front();
    // create argument for the parameter
    prog_params.add(input, migraphx::argument(param_shapes[input], inputTensorValues.data()));

    
    // auto t2 = std::chrono::steady_clock::now();
    // run inference
    auto outputs = net.eval(prog_params);

    // auto t3 = std::chrono::steady_clock::now();

    std::vector<bbox_t> candidates;
    float* out = reinterpret_cast<float*>(outputs[0].data());
    for (size_t i = 0; i < 15120; i++)
    {
        const float* box_buffer = out+20*i;
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
        det.push_back(box1);
    }

    // auto t4 = std::chrono::steady_clock::now();

    // auto pre_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    // auto infer_time = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    // auto post_time = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();

    // printf("[ONNX] Pre: %ld us|Infer: %ld us|Post: %ld us\n",
    //        pre_time, infer_time, post_time);
}
