#include "num.hpp"

int DigitalRecognition::matToDigital(cv::Mat &img)
    {
        img.convertTo(img, CV_32FC1, 1.0f / 255.0f);
        cv::resize(img, img, cv::Size(IMAGE_COLS, IMAGE_ROWS));
        auto input_tensor = torch::from_blob(img.data, {1, IMAGE_COLS, IMAGE_ROWS, 1});
        input_tensor = input_tensor.permute({0, 3, 1, 2}).to(device);
        std::vector<torch::jit::IValue> inputs;
        inputs.emplace_back(input_tensor);
        // auto start = system_clock::now();
        at::Tensor output = module.forward(inputs).toTensor();
        int ans = output.argmax(1).item().toInt();
        // auto end = system_clock::now();
        // auto duration = duration_cast<microseconds>(end - start);
        // cout << "avg_Spent" << (double(duration.count()) * microseconds::period::num / microseconds::period::den) << " seconds." << endl;
        return ans;
    }

