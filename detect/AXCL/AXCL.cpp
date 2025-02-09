//
// Inherit from SJTU-CV-2021/autoaim/detector/AXCL.hpp commit 7093b430 Harry-hhj on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Manage TRT Inference
//

#include "AXCL.hpp"
#include <string>
#include <fstream>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>

#define SAMPLE_AX_ENGINE_DEAL_HANDLE            \
    if (0 != ret)                               \
    {                                           \
        return AX_ENGINE_DestroyHandle(handle); \
    }

#define SAMPLE_AX_ENGINE_DEAL_HANDLE_IO         \
    if (0 != ret)                               \
    {                                           \
        free_io(&io_data);          \
        return AX_ENGINE_DestroyHandle(handle); \
    }

void free_io(AX_ENGINE_IO_T* io)
{
    for (size_t j = 0; j < io->nInputSize; ++j)
    {
        AX_ENGINE_IO_BUFFER_T* pBuf = io->pInputs + j;
        AX_SYS_MemFree(pBuf->phyAddr, pBuf->pVirAddr);
    }
    for (size_t j = 0; j < io->nOutputSize; ++j)
    {
        AX_ENGINE_IO_BUFFER_T* pBuf = io->pOutputs + j;
        AX_SYS_MemFree(pBuf->phyAddr, pBuf->pVirAddr);
    }
    delete[] io->pInputs;
    delete[] io->pOutputs;
}
bool read_file(const std::string& path, std::vector<char>& data)
{
    std::fstream fs(path, std::ios::in | std::ios::binary);

    if (!fs.is_open())
    {
        return false;
    }

    fs.seekg(std::ios::end);
    auto fs_end = fs.tellg();
    fs.seekg(std::ios::beg);
    auto fs_beg = fs.tellg();

    auto file_size = static_cast<size_t>(fs_end - fs_beg);
    auto vector_size = data.size();

    data.reserve(vector_size + file_size);
    data.insert(data.end(), std::istreambuf_iterator<char>(fs), std::istreambuf_iterator<char>());

    fs.close();

    return true;
}

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

AXCL::AXCL(const std::string &AXCL_file) : BackEnd() 
{
    std::filesystem::path AXCL_file_path(AXCL_file);
    AX_ENGINE_NPU_ATTR_T npu_attr;
    memset(&npu_attr, 0, sizeof(npu_attr));
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    auto ret = AX_ENGINE_Init(&npu_attr);

    if (0 != ret)
    {
        return ret;
    }

    // 2. load model
    std::vector<char> model_buffer;
    if (!read_file(AXCL_file, model_buffer))
    {
        fprintf(stderr, "Read Run-Joint model(%s) file failed.\n", AXCL_file.c_str());
        return false;
    }

    // 3. create handle
    ret = AX_ENGINE_CreateHandle(&handle, model_buffer.data(), model_buffer.size());
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    fprintf(stdout, "Engine creating handle is done.\n");

    // 4. create context
    ret = AX_ENGINE_CreateContext(handle);
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    fprintf(stdout, "Engine creating context is done.\n");

    // 5. set io
    AX_ENGINE_IO_INFO_T* io_info;
    ret = AX_ENGINE_GetIOInfo(handle, &io_info);
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    fprintf(stdout, "Engine get io info is done. \n");

    for (int i = 0; i < io_info->nOutputSize; i++)
    {
        fprintf(stdout, "%d: name=%s,shape=%d,%d,%d  \n", i, io_info->pOutputs[i].pName, io_info->pOutputs[i].pShape[0], io_info->pOutputs[i].pShape[1], io_info->pOutputs[i].pShape[2]);
    }

    // 6. alloc io
    ret = middleware::prepare_io(io_info, &io_data, std::make_pair(AX_ENGINE_ABST_DEFAULT, AX_ENGINE_ABST_CACHED));
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    fprintf(stdout, "Engine alloc io is done. \n");
}

AXCL::~AXCL()
{
    free_io(&io_data);
    return AX_ENGINE_DestroyHandle(handle);
}

void AXCL::operator()(const cv::Mat &src, std::vector<bbox_t> &det)
{
    // pre-process [bgr2rgb & resize]
    det.clear();
    cv::Mat img_new(384, 640, CV_8UC3, image.data());
    float fx = (float)src.cols / 640.f, fy = (float)src.rows / 384.f;
    
    if (src.cols != 640 || src.rows != 384)
    {
        cv::resize(src, src, {640, 384});
    }
    cv::cvtColor(src, x, cv::COLOR_BGR2RGB);
    // 7. insert input
    memcpy(io_data.Inputs[0].pVirAddr, inputTensorValues.data(), inputTensorValues.size());
    
    auto ret = AX_ENGINE_RunSync(handle, &io_data);
    SAMPLE_AX_ENGINE_DEAL_HANDLE_IO

    std::vector<bbox_t> candidates;
    float* out = (float*)io_data.pOutputs[0].pVirAddr;
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
}
