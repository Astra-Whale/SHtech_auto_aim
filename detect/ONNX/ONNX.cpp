#include "ONNX.hpp"
#include <filesystem>
#include <opencv2/dnn/dnn.hpp>
#include <algorithm>
#include <cmath>

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

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
    det.clear();

    if (src.cols != INPUT_W || src.rows != INPUT_H) {
        LOGW_S("[ONNX_ROCM]Warning: preprocess output size mismatch, expected %d %d but got %d %d",
               INPUT_W, INPUT_H, src.cols, src.rows);
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(src, blob, 1.0 / 255.0, cv::Size(), cv::Scalar(0, 0, 0), false, false);

    if (blob.isContinuous()) {
        inputTensorValues.assign(blob.begin<float>(), blob.end<float>());
    } else {
        cv::Mat continuous_mat = blob.clone();
        inputTensorValues.assign(continuous_mat.begin<float>(), continuous_mat.end<float>());
    }

    migraphx::program_parameters prog_params;
    auto param_shapes = net.get_parameter_shapes();
    auto input_name = param_shapes.names().front();

    prog_params.add(input_name, migraphx::argument(param_shapes[input_name], inputTensorValues.data()));

    auto outputs = net.eval(prog_params);
    float* output_data = reinterpret_cast<float*>(outputs[0].data());

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

        if (ptr[8] < LOGIT_THRESH) {
            continue;
        }

        float conf = sigmoid(ptr[8]);

        int color_id = 0;
        float max_color_val = ptr[9];
        for (int c = 1; c < 4; c++) {
            if (ptr[9 + c] > max_color_val) {
                max_color_val = ptr[9 + c];
                color_id = c;
            }
        }
        if (color_id == 2 || color_id == 3) {
            continue;
        }

        int class_id = 0;
        float max_class_val = ptr[13];
        for (int c = 1; c < 9; c++) {
            if (ptr[13 + c] > max_class_val) {
                max_class_val = ptr[13 + c];
                class_id = c;
            }
        }

        if (class_id == 7 || class_id == 8) {
            class_id = 9;
        } else if (class_id == 0) {
            class_id = 7;
        } else if (class_id == 6) {
            class_id = 8;
        }

        if (color_id == 0) {
            color_id = 1;
        } else if (color_id == 1) {
            color_id = 0;
        }

        bbox_t box;
        box.confidence = conf;
        box.color_id = color_id;
        box.tag_id = class_id;
        box.source = DetectionSource::NEURAL_NETWORK;

        float x_min = 1e5f;
        float y_min = 1e5f;
        float x_max = -1e5f;
        float y_max = -1e5f;

        for (int k = 0; k < 4; k++) {
            float x = ptr[2 * k];
            float y = ptr[2 * k + 1];

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
        boxes_nms.push_back(cv::Rect(x_min - 0.1f * w, y_min - 0.1f * h, w * 1.2f, h * 1.2f));
        scores_nms.push_back(conf);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_nms, scores_nms, CONF_THRESH, NMS_THRESH, indices);

    det.reserve(indices.size());
    for (int idx : indices) {
        det.push_back(temp_bboxes[idx]);
    }

    std::sort(det.begin(), det.end(), std::greater<bbox_t>());
}
