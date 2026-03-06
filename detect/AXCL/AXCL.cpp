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
        AX_ENGINE_DestroyHandle(handle);        \
        return;                                 \
    }

#define SAMPLE_AX_ENGINE_DEAL_HANDLE_IO         \
    if (0 != ret)                               \
    {                                           \
        free_io(&io_data);                      \
        AX_ENGINE_DestroyHandle(handle);        \
        return;                                 \
    }

#define AX_CMM_ALIGN_SIZE 128

typedef enum
{
    AX_ENGINE_ABST_DEFAULT = 0,
    AX_ENGINE_ABST_CACHED = 1,
} AX_ENGINE_ALLOC_BUFFER_STRATEGY_T;

typedef std::pair<AX_ENGINE_ALLOC_BUFFER_STRATEGY_T, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T> INPUT_OUTPUT_ALLOC_STRATEGY;

const char* AX_CMM_SESSION_NAME = "ax-samples-cmm";

void free_io_index(AX_ENGINE_IO_BUFFER_T* io_buf, size_t index)
{
    for (int i = 0; i < (int)index; ++i)
    {
        AX_ENGINE_IO_BUFFER_T* pBuf = io_buf + i;
        AX_SYS_MemFree(pBuf->phyAddr, pBuf->pVirAddr);
    }
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

static inline int prepare_io(AX_ENGINE_IO_INFO_T* info, AX_ENGINE_IO_T* io_data, INPUT_OUTPUT_ALLOC_STRATEGY strategy)
{
    memset(io_data, 0, sizeof(*io_data));
    io_data->pInputs = new AX_ENGINE_IO_BUFFER_T[info->nInputSize];
    memset(io_data->pInputs, 0, sizeof(AX_ENGINE_IO_BUFFER_T) * info->nInputSize);
    io_data->nInputSize = info->nInputSize;

    auto ret = 0;
    for (int i = 0; i < (int)info->nInputSize; ++i)
    {
        auto meta = info->pInputs[i];
        auto buffer = &io_data->pInputs[i];
        if (strategy.first == AX_ENGINE_ABST_CACHED)
        {
            ret = AX_SYS_MemAllocCached((AX_U64*)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8*)(AX_CMM_SESSION_NAME));
        }
        else
        {
            ret = AX_SYS_MemAlloc((AX_U64*)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8*)(AX_CMM_SESSION_NAME));
        }

        if (ret != 0)
        {
            free_io_index(io_data->pInputs, i);
            LOGE_S( "[AXCL] Allocate input{%d} { phy: %p, vir: %p, size: %lu Bytes }. fail \n", i, (void*)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
            return ret;
        }
        // LOGE_S( "[AXCL] Allocate input{%d} { phy: %p, vir: %p, size: %lu Bytes }. \n", i, (void*)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
    }

    io_data->pOutputs = new AX_ENGINE_IO_BUFFER_T[info->nOutputSize];
    memset(io_data->pOutputs, 0, sizeof(AX_ENGINE_IO_BUFFER_T) * info->nOutputSize);
    io_data->nOutputSize = info->nOutputSize;
    for (int i = 0; i < (int)info->nOutputSize; ++i)
    {
        auto meta = info->pOutputs[i];
        auto buffer = &io_data->pOutputs[i];
        buffer->nSize = meta.nSize;
        if (strategy.second == AX_ENGINE_ABST_CACHED)
        {
            ret = AX_SYS_MemAllocCached((AX_U64*)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8*)(AX_CMM_SESSION_NAME));
        }
        else
        {
            ret = AX_SYS_MemAlloc((AX_U64*)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8*)(AX_CMM_SESSION_NAME));
        }
        if (ret != 0)
        {
            LOGE_S( "[AXCL] Allocate output{%d} { phy: %p, vir: %p, size: %lu Bytes }. fail \n", i, (void*)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
            free_io_index(io_data->pInputs, io_data->nInputSize);
            free_io_index(io_data->pOutputs, i);
            return ret;
        }
        // LOGE_S( "[AXCL] Allocate output{%d} { phy: %p, vir: %p, size: %lu Bytes }.\n", i, (void*)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
    }

    return 0;
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

static const LayerConfig layer_configs[3] = {
    {64, 80, {{10, 13}, {16, 30}, {33, 23}}, 8},
    {32, 40, {{30, 61}, {62, 45}, {59, 119}}, 16},
    {16, 20, {{116, 90}, {156, 198}, {373, 326}}, 32}
};

AXCL::AXCL(const std::string &AXCL_file) : BackEnd(), inputTensorValues(3*512*640, 0)
{
    AX_SYS_Init();
    AX_ENGINE_NPU_ATTR_T npu_attr;
    memset(&npu_attr, 0, sizeof(npu_attr));
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    auto ret = AX_ENGINE_Init(&npu_attr);

    if (0 != ret)
    {
        LOGE_S( "[AXCL] Init ENGINE failed.\n");
        return;
    }

    // 2. load model
    if (!read_file(AXCL_file, model_buffer))
    {
        LOGE_S( "[AXCL] Read Run-Joint model(%s) file failed.\n", AXCL_file.c_str());
        return;
    }

    // 3. create handle
    ret = AX_ENGINE_CreateHandle(&handle, model_buffer.data(), model_buffer.size());
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    LOGM_S( "[AXCL] Engine creating handle is done.\n");

    // 4. create context
    ret = AX_ENGINE_CreateContext(handle);
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    LOGM_S( "[AXCL] Engine creating context is done.\n");

    // 5. set io
    AX_ENGINE_IO_INFO_T* io_info;
    ret = AX_ENGINE_GetIOInfo(handle, &io_info);
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    LOGM_S( "[AXCL] Engine get io info is done. \n");

    for (int i = 0; i < io_info->nOutputSize; i++)
    {
        LOGM_S( "[AXCL] %d: name=%s,shape=%d,%d,%d  \n", i, io_info->pOutputs[i].pName, io_info->pOutputs[i].pShape[0], io_info->pOutputs[i].pShape[1], io_info->pOutputs[i].pShape[2]);
    }

    // 6. alloc io
    ret = prepare_io(io_info, &io_data, std::make_pair(AX_ENGINE_ABST_DEFAULT, AX_ENGINE_ABST_CACHED));
    SAMPLE_AX_ENGINE_DEAL_HANDLE
    LOGM_S( "[AXCL] Engine alloc io is done. \n");
}

AXCL::~AXCL()
{
    free_io(&io_data);
    AX_ENGINE_DestroyHandle(handle);
    AX_ENGINE_Deinit();
    AX_SYS_Deinit();
}

void AXCL::operator()(const cv::Mat &src, std::vector<bbox_t> &det)
{
    // pre-process [bgr2rgb & resize]
    det.clear();
    cv::Mat img_new(512, 640, CV_8UC3, inputTensorValues.data());
    constexpr float inv_w = 1.0f / 640.f, inv_h = 1.0f / 512.f;
    const float fx = (float)src.cols * inv_w, fy = (float)src.rows * inv_h;
    
    if (src.cols != 640 || src.rows != 512)
    {
        cv::resize(src, img_new, {640, 512});
    }
    else
    {
        src.copyTo(img_new);
    }
    // 7. insert input
    memcpy(io_data.pInputs[0].pVirAddr, inputTensorValues.data(), inputTensorValues.size());
    
    auto ret = AX_ENGINE_RunSync(handle, &io_data);
    SAMPLE_AX_ENGINE_DEAL_HANDLE_IO

    std::vector<bbox_t> candidates;
    candidates.reserve(512);

    const float* out = (float*)io_data.pOutputs[0].pVirAddr;
    constexpr float thres_logit = inv_sigmoid(KEEP_THRES);
    constexpr int max_candidates = 500;

    int global_idx = 0;

    // 三层anchor-based解码
    for (int l = 0; l < 3; ++l) {
        const auto& conf = layer_configs[l];
        const float s_fx = conf.stride * fx;
        const float s_fy = conf.stride * fy;

        for (int a = 0; a < 3; ++a) {
            const float a_w_fx = conf.anchors[a][0] * fx;
            const float a_h_fy = conf.anchors[a][1] * fy;

            for (int y = 0; y < conf.grid_h; ++y) {
                const float py = y * s_fy;

                for (int x = 0; x < conf.grid_w; ++x) {
                    const float* box_buffer = out + 22 * global_idx;
                    global_idx++;

                    // 预过滤：置信度
                    if (box_buffer[8] < thres_logit) continue;

                    // 预过滤：颜色逻辑映射
                    int color_id = argmax(box_buffer + 9, 4);
                    if (color_id == 2 || color_id == 3) continue;

                    if ((int)candidates.size() >= max_candidates) {
                        LOGW_S("[AXCL] Candidate limit reached (%d), stopping early detection.\n", max_candidates);
                        goto decode_done;
                    }

                    candidates.emplace_back();
                    auto &box = candidates.back();
                    
                    const float px = x * s_fx;

                    // 还原4个关键点
                    for (int k = 0; k < 4; k++) {
                        box.pts[k].x = box_buffer[2 * k] * a_w_fx + px;
                        box.pts[k].y = box_buffer[2 * k + 1] * a_h_fy + py;
                    }

                    box.confidence = sigmoid(box_buffer[8]);
                    box.color_id = (color_id == 0) ? 1 : 0;

                    // 类别标签映射
                    int class_id = argmax(box_buffer + 13, 9);
                    if (class_id == 7 || class_id == 8) class_id = 0;
                    else if (class_id == 0) class_id = 7;
                    box.tag_id = class_id;
                }
            }
        }
    }

decode_done:
    // 排序与NMS
    std::sort(candidates.begin(), candidates.end(), std::greater<bbox_t>());

    det.clear();
    std::vector<bool> removed(candidates.size(), false);

    for (int i = 0; i < (int)candidates.size() && (int)det.size() < TOPK_NUM; i++) {
        if (removed[i]) continue;
        det.push_back(candidates[i]);

        for (int j = i + 1; j < (int)candidates.size(); j++) {
            if (!removed[j] && is_overlap(candidates[i], candidates[j])) {
                removed[j] = true;
            }
        }
    }
}
