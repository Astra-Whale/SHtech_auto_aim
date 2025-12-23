#include "ONNX.hpp"
#include <filesystem>
#include <opencv2/dnn/dnn.hpp> // For NMSBoxes
#include <algorithm>
#include <cmath>

// ================= 辅助函数 =================

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// 获取 Letterbox 变换矩阵 (保持长宽比)
static cv::Mat get_transform_matrix(const cv::Size& src_size, const cv::Size& dst_size, ONNX::PreProcessParams& params) {
    float scale = std::min((float)dst_size.width / src_size.width, (float)dst_size.height / src_size.height);
    
    // 计算居中填充的偏移量
    float ox = (dst_size.width - src_size.width * scale) * 0.5f;
    float oy = (dst_size.height - src_size.height * scale) * 0.5f;
    
    params.scale = scale;
    params.ox = ox;
    params.oy = oy;
    
    return (cv::Mat_<float>(2, 3) << scale, 0, ox, 0, scale, oy);
}

// ================= 类实现 =================

ONNX::ONNX(const std::string &onnx_file) : BackEnd() 
{
    std::filesystem::path onnx_file_path(onnx_file);
    auto cache_file_path = onnx_file_path;
    cache_file_path.replace_extension("cache");
    
    if (std::filesystem::exists(cache_file_path)) {
        build_engine_from_cache(cache_file_path.c_str());
    } else {
        build_engine_from_onnx(onnx_file_path.c_str());
        cache_engine(cache_file_path.c_str());
    }
}

void ONNX::build_engine_from_onnx(const std::string &onnx_file)
{
    migraphx::onnx_options onnx_opts;
    net = parse_onnx(onnx_file.c_str(), onnx_opts);
    
    migraphx::target targ = migraphx::target("gpu");
    migraphx::compile_options comp_opts;
    comp_opts.set_offload_copy();
    migraphx::quantize_fp16(net); // 开启 FP16
    net.compile(targ, comp_opts);
}

void ONNX::build_engine_from_cache(const std::string &cache_file_path)
{
    net = migraphx::load(cache_file_path.c_str());
}

void ONNX::cache_engine(const std::string &cache_file_path)
{
    migraphx::save(net, cache_file_path.c_str());
}

ONNX::~ONNX() {}

void ONNX::operator()(const cv::Mat &src, std::vector<bbox_t> &det)
{
    det.clear();
    
    // ================= 1. Pre-process (Letterbox) =================
    PreProcessParams params;
    cv::Mat trans_mat = get_transform_matrix(src.size(), cv::Size(INPUT_W, INPUT_H), params);
    
    cv::Mat warped;
    // 使用 114 填充背景 (YOLO 标准)
    cv::warpAffine(src, warped, trans_mat, cv::Size(INPUT_W, INPUT_H), 
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    
    cv::Mat blob;
    // BGR -> RGB, /255.0, HWC -> CHW
    cv::dnn::blobFromImage(warped, blob, 1.0/255.0, cv::Size(), cv::Scalar(0,0,0), true, false);
    
    inputTensorValues.assign(blob.begin<float>(), blob.end<float>());

    // ================= 2. Inference =================
    migraphx::program_parameters prog_params;
    auto param_shapes = net.get_parameter_shapes();
    auto input_name = param_shapes.names().front();
    
    prog_params.add(input_name, migraphx::argument(param_shapes[input_name], inputTensorValues.data()));
    
    auto outputs = net.eval(prog_params);
    float* output_data = reinterpret_cast<float*>(outputs[0].data());

    // ================= 3. Post-process =================
    // 假设 Output Shape: [1, 25200, 22]
    const int num_anchors = 25200;
    const int stride = 22;
    
    std::vector<cv::Rect> boxes_nms;     // NMS 用的矩形框
    std::vector<float> scores_nms;       // NMS 用的分数
    std::vector<bbox_t> temp_bboxes;     // 暂存解码后的完整对象
    
    boxes_nms.reserve(128);
    scores_nms.reserve(128);
    temp_bboxes.reserve(128);

    for (int i = 0; i < num_anchors; i++) {
        float* ptr = output_data + i * stride;
        
        // 1. Fail-Fast 过滤: 检查置信度 Logit
        if (ptr[8] < LOGIT_THRESH) continue;
        
        float conf = sigmoid(ptr[8]);
        
        // 2. 颜色分类 (Blue, Red, Gray, Purple) -> Index 9-12
        int color_id = 0;
        float max_color_val = ptr[9];
        for(int c=1; c<4; c++) {
            if(ptr[9+c] > max_color_val) {
                max_color_val = ptr[9+c];
                color_id = c;
            }
        }
        // 过滤灰色(2)和紫色(3)
        if (color_id == 2 || color_id == 3) continue;

        // 3. 数字分类 (G, 1-5, O, Bs, Bb) -> Index 13-21
        int class_id = 0;
        float max_class_val = ptr[13];
        for(int c=1; c<9; c++) {
            if(ptr[13+c] > max_class_val) {
                max_class_val = ptr[13+c];
                class_id = c;
            }
        }

        // 深大的定义：G（哨兵）1（一号）2（二号）3（三号）4（四号）5（五号）O（前哨站）Bs（基地）Bb（基地大装甲）
        // 我们的定义：B（基地）1（一号）2（二号）3（三号）4（四号）5（五号）O（前哨站）G（哨兵）

        if (class_id == 7||class_id == 8) class_id = 0; // B
        else if (class_id == 0) class_id = 7; // G

        // 深大的定义：蓝红灰紫
        // 我们的定义：0：红色 1：蓝色
        
        if (color_id == 0) color_id = 1; // Red
        else if (color_id == 1) color_id = 0; // Blue
        
        // 4. 填充 bbox_t 结构体
        bbox_t box;
        box.confidence = conf;
        box.color_id = color_id;
        box.tag_id = class_id;
        box.source = DetectionSource::NEURAL_NETWORK;
        
        // 提取关键点并计算 NMS 用的包围盒
        float x_min = 1e5, y_min = 1e5;
        float x_max = -1e5, y_max = -1e5;
        
        // ptr[0-7] 是 4个点的 x,y 坐标
        for (int k = 0; k < 4; k++) {
            float x = ptr[2*k];
            float y = ptr[2*k+1];
            
            // 正确填充 cv::Point2f 数组
            box.pts[k].x = x;
            box.pts[k].y = y;
            
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }
        
        // 临时保存对象
        temp_bboxes.push_back(box);
        
        // 准备 NMS 数据 (扩大一点 ROI 以防止过度抑制)
        float w = x_max - x_min;
        float h = y_max - y_min;
        boxes_nms.push_back(cv::Rect(x_min - 0.1f*w, y_min - 0.1f*h, w*1.2f, h*1.2f));
        scores_nms.push_back(conf);
    }
    
    // ================= 4. NMS =================
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_nms, scores_nms, CONF_THRESH, NMS_THRESH, indices);
    
    // ================= 5. Coordinate Restoration =================
    det.reserve(indices.size());
    for (int idx : indices) {
        bbox_t& box = temp_bboxes[idx];
        
        // 使用预存的参数进行逆仿射变换
        // 公式: real_x = (pred_x - ox) / scale
        for (int k = 0; k < 4; k++) {
            box.pts[k].x = (box.pts[k].x - params.ox) / params.scale;
            box.pts[k].y = (box.pts[k].y - params.oy) / params.scale;
        }
        
        det.push_back(box);
    }
    
    // 如果外部需要按置信度排序
    // bbox_t 重载了 > 运算符 (confidence > a.confidence)
    std::sort(det.begin(), det.end(), std::greater<bbox_t>());
}