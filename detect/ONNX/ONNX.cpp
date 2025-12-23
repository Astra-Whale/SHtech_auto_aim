#include "ONNX.hpp"
#include <filesystem>
#include <opencv2/dnn/dnn.hpp> // For NMSBoxes
#include <algorithm>
#include <cmath>

// ================= 辅助函数 =================

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// 获取 Letterbox 的仿射变换矩阵
static cv::Mat get_transform_matrix(const cv::Size& src_size, const cv::Size& dst_size, ONNX::PreProcessParams& params) {
    float scale = std::min((float)dst_size.width / src_size.width, (float)dst_size.height / src_size.height);
    
    float ox = (dst_size.width - src_size.width * scale) * 0.5f;
    float oy = (dst_size.height - src_size.height * scale) * 0.5f;
    
    params.scale = scale;
    params.ox = ox;
    params.oy = oy;
    
    // [scale, 0, ox]
    // [0, scale, oy]
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
    
    // 启用 FP16 加速
    migraphx::quantize_fp16(net);
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
    // 使用 warpAffine 一步完成 resize + padding
    // 114 是 YOLO 训练常用的 padding 值
    cv::warpAffine(src, warped, trans_mat, cv::Size(INPUT_W, INPUT_H), 
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    
    // HWC -> CHW, BGR -> RGB, Normalize 0-1
    // blobFromImage 内部有优化
    cv::Mat blob;
    cv::dnn::blobFromImage(warped, blob, 1.0/255.0, cv::Size(), cv::Scalar(0,0,0), true, false);
    
    // 填充输入 Buffer
    inputTensorValues.assign(blob.begin<float>(), blob.end<float>());

    // ================= 2. Inference (MIGraphX) =================
    migraphx::program_parameters prog_params;
    auto param_shapes = net.get_parameter_shapes();
    auto input_name = param_shapes.names().front();
    
    prog_params.add(input_name, migraphx::argument(param_shapes[input_name], inputTensorValues.data()));
    
    // 同步执行
    auto outputs = net.eval(prog_params);
    float* output_data = reinterpret_cast<float*>(outputs[0].data());

    // ================= 3. Post-process (Decoding) =================
    // 假设输出形状: [1, 25200, 22]
    // 22 channels: [x0,y0...x3,y3 (8), conf(1), color(4), class(9)]
    const int num_anchors = 25200; // YOLO stride 8,16,32 --> 640x640 -> 25200
    const int stride = 22;
    
    std::vector<cv::Rect> boxes_nms;
    std::vector<float> scores_nms;
    std::vector<bbox_t> temp_bboxes;
    
    // 预分配内存以提升性能
    boxes_nms.reserve(100);
    scores_nms.reserve(100);
    temp_bboxes.reserve(100);

    for (int i = 0; i < num_anchors; i++) {
        float* ptr = output_data + i * stride;
        
        // --- Fail-Fast 优化 ---
        // 直接检查 Logit 值，避免 exp 计算
        if (ptr[8] < LOGIT_THRESH) continue;
        
        // 计算实际置信度
        float conf = sigmoid(ptr[8]);
        
        // --- 颜色分类 (0-3) ---
        // ptr[9:13] -> Blue, Red, Gray, Purple
        // 找到最大值的索引
        int color_id = 0;
        float max_color_val = ptr[9];
        for(int c=1; c<4; c++) {
            if(ptr[9+c] > max_color_val) {
                max_color_val = ptr[9+c];
                color_id = c;
            }
        }
        
        // 过滤灰色(2)和紫色(3)，或者根据需求过滤
        if (color_id == 2 || color_id == 3) continue;

        // --- 数字分类 (0-8) ---
        // ptr[13:22]
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

        // --- 组装 bbox_t ---
        bbox_t box;
        box.confidence = conf;
        box.color_id = color_id;
        box.tag_id = class_id;

        
        
        // 提取关键点 (此时还是 640x640 下的坐标)
        // 存储顺序: P0(x,y), P1(x,y), P2(x,y), P3(x,y)
        float x_min = 10000, y_min = 10000;
        float x_max = -10000, y_max = -10000;
        
        for (int k = 0; k < 4; k++) {
            box.pts[2*k] = ptr[2*k];     // x
            box.pts[2*k+1] = ptr[2*k+1]; // y
            
            x_min = std::min(x_min, box.pts[2*k]);
            x_max = std::max(x_max, box.pts[2*k]);
            y_min = std::min(y_min, box.pts[2*k+1]);
            y_max = std::max(y_max, box.pts[2*k+1]);
        }
        
        // 计算 NMS 用的包围盒 (扩大 ROI 策略)
        float w = x_max - x_min;
        float h = y_max - y_min;
        
        // 稍微扩大一点框给 NMS 用，防止太紧凑
        cv::Rect rect;
        rect.x = x_min - 0.2f * w;
        rect.y = y_min - 0.2f * h;
        rect.width = w * 1.4f;
        rect.height = h * 1.4f;
        
        temp_bboxes.push_back(box);
        boxes_nms.push_back(rect);
        scores_nms.push_back(conf); // 这里也可以用 conf * class_score
    }
    
    // ================= 4. NMS =================
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_nms, scores_nms, CONF_THRESH, NMS_THRESH, indices);
    
    // ================= 5. Coordinate Restoration =================
    // 仅对留下来的目标进行逆变换
    det.reserve(indices.size());
    for (int idx : indices) {
        bbox_t& box = temp_bboxes[idx];
        
        // Inverse Affine: (x - ox) / scale
        for (int k = 0; k < 4; k++) {
            box.pts[2*k]     = (box.pts[2*k] - params.ox) / params.scale;
            box.pts[2*k+1]   = (box.pts[2*k+1] - params.oy) / params.scale;
        }
        
        det.push_back(box);
    }
}