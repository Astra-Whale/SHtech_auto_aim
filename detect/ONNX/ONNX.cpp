#include "ONNX.hpp"
#include <filesystem>
#include <opencv2/dnn/dnn.hpp> // For NMSBoxes
#include <algorithm>
#include <cmath>

// ================= 辅助函数 =================

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
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
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    det.clear();
    
    if (src.rows * INPUT_W != src.cols * INPUT_H) {

        LOGW_S("[ONNX_ROCM]Warning: Input image aspect ratio differs from model input!");
        LOGW_S("%d %d %d %d",src.rows,INPUT_W,src.cols,INPUT_H);
    
    }

    // ================= 1. Pre-process (Direct Resize) =================
    
    cv::Mat resized_img;
    cv::resize(src, resized_img, cv::Size(INPUT_W, INPUT_H));

    cv::Mat blob; // shape will be [1, 3, 512, 640]

    cv::dnn::blobFromImage(
        resized_img,                // 输入图像
        blob,                   // 输出 blob
        1.0 / 255.0,            // 缩放因子（归一化）
        cv::Size(),             // 不再调整尺寸（已 resize）
        cv::Scalar(0, 0, 0),    // 均值减去（这里为 0）
        true,                   // swapRB = true → BGR 转 RGB
        false                   // crop = false
    );

    if (blob.isContinuous()) {
        inputTensorValues.assign(blob.begin<float>(), blob.end<float>());
    } else {
        // 防御性代码，防止非常规操作导致的内存不连续
        cv::Mat continuous_mat = blob.clone();
        inputTensorValues.assign(continuous_mat.begin<float>(), continuous_mat.end<float>());
    }

    //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //std::cout << "Pre-process time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;

    // ================= 2. Inference =================
    //std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    migraphx::program_parameters prog_params;
    auto param_shapes = net.get_parameter_shapes();
    auto input_name = param_shapes.names().front();
    
    prog_params.add(input_name, migraphx::argument(param_shapes[input_name], inputTensorValues.data()));
    
    auto outputs = net.eval(prog_params);
    float* output_data = reinterpret_cast<float*>(outputs[0].data());

    //std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    //std::cout << "Inference time: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << " us" << std::endl;

    // ================= 3. Post-process =================
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    // Output Shape: [1, 20160, 22], as we cut down the input size from szu's 640*640 to H512W640
    const int num_anchors = 20160;
    const int stride = 22;
    
    std::vector<cv::Rect> boxes_nms;
    std::vector<float> scores_nms;
    std::vector<bbox_t> temp_bboxes;
    
    boxes_nms.reserve(128);
    scores_nms.reserve(128);
    temp_bboxes.reserve(128);

    for (int i = 0; i < num_anchors; i++) {
        float* ptr = output_data + i * stride;
        
        if (ptr[8] < LOGIT_THRESH) continue;
        
        float conf = sigmoid(ptr[8]);
        
        int color_id = 0;
        float max_color_val = ptr[9];
        for(int c=1; c<4; c++) {
            if(ptr[9+c] > max_color_val) {
                max_color_val = ptr[9+c];
                color_id = c;
            }
        }
        if (color_id == 2 || color_id == 3) continue;

        int class_id = 0;
        float max_class_val = ptr[13];
        for(int c=1; c<9; c++) {
            if(ptr[13+c] > max_class_val) {
                max_class_val = ptr[13+c];
                class_id = c;
            }
        }

        if (class_id == 7||class_id == 8) class_id = 0; // B
        else if (class_id == 0) class_id = 7; // G

        if (color_id == 0) color_id = 1; // Red
        else if (color_id == 1) color_id = 0; // Blue
        
        bbox_t box;
        box.confidence = conf;
        box.color_id = color_id;
        box.tag_id = class_id;
        box.source = DetectionSource::NEURAL_NETWORK;
        
        float x_min = 1e5, y_min = 1e5;
        float x_max = -1e5, y_max = -1e5;
        
        for (int k = 0; k < 4; k++) {
            float x = ptr[2*k];
            float y = ptr[2*k+1];
            
            box.pts[k].x = x;
            box.pts[k].y = y;
            
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }
        
        temp_bboxes.push_back(box);
        
        float w = x_max - x_min;
        float h = y_max - y_min;
        boxes_nms.push_back(cv::Rect(x_min - 0.1f*w, y_min - 0.1f*h, w*1.2f, h*1.2f));
        scores_nms.push_back(conf);
    }
    
    // ================= 4. NMS =================
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_nms, scores_nms, CONF_THRESH, NMS_THRESH, indices);
    
    // ================= 5. Coordinate Restoration (Hardcoded Resize) =================
    
    // 计算宽和高的缩放比例
    float r_w = (float)src.cols / INPUT_W;
    float r_h = (float)src.rows / INPUT_H;

    det.reserve(indices.size());
    for (int idx : indices) {
        bbox_t& box = temp_bboxes[idx];
        
        // 直接按比例映射回原图坐标
        for (int k = 0; k < 4; k++) {
            box.pts[k].x *= r_w;
            box.pts[k].y *= r_h;
        }
        
        det.push_back(box);
    }
    
    std::sort(det.begin(), det.end(), std::greater<bbox_t>());
    //std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    //std::cout << "Post-process time: " << std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() << " us" << std::endl;
}