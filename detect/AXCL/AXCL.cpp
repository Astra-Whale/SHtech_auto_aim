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

#define AX_CMM_ALIGN_SIZE 128

typedef enum
{
    AX_ENGINE_ABST_DEFAULT = 0,
    AX_ENGINE_ABST_CACHED = 1,
} AX_ENGINE_ALLOC_BUFFER_STRATEGY_T;

typedef std::pair<AX_ENGINE_ALLOC_BUFFER_STRATEGY_T, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T> INPUT_OUTPUT_ALLOC_STRATEGY;

const char* AX_CMM_SESSION_NAME = "ax-samples-cmm";

namespace
{
void release_io_buffer(AX_ENGINE_IO_BUFFER_T* buffer)
{
    if (buffer == nullptr)
    {
        return;
    }

    if (buffer->phyAddr != 0 || buffer->pVirAddr != nullptr)
    {
        AX_SYS_MemFree(buffer->phyAddr, buffer->pVirAddr);
        buffer->phyAddr = 0;
        buffer->pVirAddr = nullptr;
    }

    buffer->nSize = 0;
}
}

void free_io_index(AX_ENGINE_IO_BUFFER_T* io_buf, size_t index)
{
    for (int i = 0; i < (int)index; ++i)
    {
        release_io_buffer(io_buf + i);
    }
}

void free_io(AX_ENGINE_IO_T* io)
{
    if (io == nullptr)
    {
        return;
    }

    if (io->pInputs != nullptr)
    {
        for (size_t j = 0; j < io->nInputSize; ++j)
        {
            release_io_buffer(io->pInputs + j);
        }
        delete[] io->pInputs;
        io->pInputs = nullptr;
    }

    if (io->pOutputs != nullptr)
    {
        for (size_t j = 0; j < io->nOutputSize; ++j)
        {
            release_io_buffer(io->pOutputs + j);
        }
        delete[] io->pOutputs;
        io->pOutputs = nullptr;
    }

    io->nInputSize = 0;
    io->nOutputSize = 0;
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
            delete[] io_data->pInputs;
            io_data->pInputs = nullptr;
            io_data->nInputSize = 0;
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
            delete[] io_data->pInputs;
            delete[] io_data->pOutputs;
            io_data->pInputs = nullptr;
            io_data->pOutputs = nullptr;
            io_data->nInputSize = 0;
            io_data->nOutputSize = 0;
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

AXCL::AXCL(const std::string &model_file) : BackEnd(), inputTensorValues(3*512*640, 0)
{
    const std::string oldExt = ".onnx";
    const std::string newExt = ".axmodel";

    std::string axcl_file = model_file;

    if (model_file.size() >= oldExt.size() &&
        model_file.compare(model_file.size() - oldExt.size(), oldExt.size(), oldExt) == 0) {
        axcl_file = model_file.substr(0, model_file.size() - oldExt.size()) + newExt;
        LOGE_S( "Read Run-Joint model(%s).\n", axcl_file.c_str());
    }

    auto ret = AX_SYS_Init();
    if (0 != ret)
    {
        LOGE_S( "[AXCL] Init SYS failed.\n");
        return;
    }
    sys_initialized = true;

    AX_ENGINE_NPU_ATTR_T npu_attr;
    memset(&npu_attr, 0, sizeof(npu_attr));
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    ret = AX_ENGINE_Init(&npu_attr);

    if (0 != ret)
    {
        LOGE_S( "[AXCL] Init ENGINE failed.\n");
        return;
    }
    engine_initialized = true;

    // 2. load model
    if (!read_file(axcl_file, model_buffer))
    {
        LOGE_S( "[AXCL] Read Run-Joint model file failed.\n");
        return;
    }

    // 3. create handle
    ret = AX_ENGINE_CreateHandle(&handle, model_buffer.data(), model_buffer.size());
    if (0 != ret)
    {
        LOGE_S( "[AXCL] Engine create handle failed.\n");
        return;
    }
    LOGM_S( "[AXCL] Engine creating handle is done.\n");

    // 4. create context
    ret = AX_ENGINE_CreateContext(handle);
    if (0 != ret)
    {
        LOGE_S( "[AXCL] Engine create context failed.\n");
        return;
    }
    LOGM_S( "[AXCL] Engine creating context is done.\n");

    // 5. set io
    AX_ENGINE_IO_INFO_T* io_info;
    ret = AX_ENGINE_GetIOInfo(handle, &io_info);
    if (0 != ret)
    {
        LOGE_S( "[AXCL] Engine get io info failed.\n");
        return;
    }
    LOGM_S( "[AXCL] Engine get io info is done. \n");

    for (int i = 0; i < io_info->nOutputSize; i++)
    {
        LOGM_S( "[AXCL] %d: name=%s,shape=%d,%d,%d  \n", i, io_info->pOutputs[i].pName, io_info->pOutputs[i].pShape[0], io_info->pOutputs[i].pShape[1], io_info->pOutputs[i].pShape[2]);
    }

    // 6. alloc io
    ret = prepare_io(io_info, &io_data, std::make_pair(AX_ENGINE_ABST_DEFAULT, AX_ENGINE_ABST_CACHED));
    if (0 != ret)
    {
        LOGE_S( "[AXCL] Engine alloc io failed.\n");
        return;
    }
    LOGM_S( "[AXCL] Engine alloc io is done. \n");

    ready = true;
}

void AXCL::releaseIo()
{
    free_io(&io_data);
    ready = false;
}

void AXCL::releaseHandle()
{
    if (handle)
    {
        AX_ENGINE_DestroyHandle(handle);
        handle = {};
    }
    ready = false;
}

void AXCL::shutdownRuntime()
{
    if (engine_initialized)
    {
        AX_ENGINE_Deinit();
        engine_initialized = false;
    }

    if (sys_initialized)
    {
        AX_SYS_Deinit();
        sys_initialized = false;
    }

    ready = false;
}

AXCL::~AXCL()
{
    releaseIo();
    releaseHandle();
    shutdownRuntime();
}

void AXCL::operator()(const cv::Mat &src, std::vector<bbox_t> &det)
{
    //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // pre-process [RGB uint8]
    det.clear();
    if (!ready)
    {
        LOGE_S("[AXCL] Inference requested before backend initialization completed.\n");
        return;
    }

    if (src.cols != 640 || src.rows != 512)
    {
        LOGW_S("[AXCL] Warning: preprocess output size mismatch, expected 640x512 but got %dx%d",
               src.cols, src.rows);
    }

    const size_t input_bytes = src.total() * src.elemSize();
    if (input_bytes != inputTensorValues.size())
    {
        LOGE_S("[AXCL] Invalid preprocess output size: expected %lu bytes but got %lu bytes",
               (unsigned long)inputTensorValues.size(),
               (unsigned long)input_bytes);
        return;
    }

    if (src.isContinuous())
    {
        memcpy(inputTensorValues.data(), src.data, inputTensorValues.size());
    }
    else
    {
        cv::Mat continuous = src.clone();
        memcpy(inputTensorValues.data(), continuous.data, inputTensorValues.size());
    }
    
    //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //std::cout << "Pre-process time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
    
    // 7. insert input
    memcpy(io_data.pInputs[0].pVirAddr, inputTensorValues.data(), inputTensorValues.size());
    
    auto ret = AX_ENGINE_RunSync(handle, &io_data);
    if (0 != ret)
    {
        LOGE_S("[AXCL] RunSync failed.\n");
        releaseIo();
        releaseHandle();
        shutdownRuntime();
        return;
    }

    std::vector<bbox_t> candidates;
    float* out = (float*)io_data.pOutputs[0].pVirAddr;
    int stride = 8, x_center = 0, y_center = 0;
    for (size_t i = 0; i < 6720; i++)
    {
        const float* box_buffer = out+21*i;
        if (box_buffer[8] >= KEEP_THRES)
        {
            candidates.emplace_back();
            auto &box = candidates.back();
            memcpy(&box.pts, box_buffer, 8 * sizeof(float));
            std::swap(box.pts[2],box.pts[3]);   // 2025、04、10系列的新模型具有和旧模型不同的角点顺序：模型输出为：左上，左下，右上，右下。现将其调整为与旧的一致：左上，左下，右下，右上
            for (auto &pt : box.pts)
            {
                pt.x = pt.x * 2 * stride + x_center;
                pt.y = pt.y * 2 * stride + y_center;
            }
            box.confidence = box_buffer[8];
            box.tag_id = argmax(box_buffer + 9, 8);
            box.color_id = argmax(box_buffer + 17, 2);
            int armor_size = argmax(box_buffer + 19, 2);
        }
        x_center += stride;
        x_center = (x_center == 640)?0:x_center;
        y_center += x_center == 0 ? stride:0;
        y_center = (y_center == 512)?0:y_center;
        stride *= x_center ==0 && y_center == 0? 2 : 1;
    }
    std::sort(candidates.begin(), candidates.end(), std::greater<bbox_t>());
    
    //std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    //std::cout << "process time: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << " us" << std::endl;
    
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
    
    //std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    //std::cout << "Post-process time: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << " us" << std::endl;
}
