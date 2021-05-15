#include <torch/script.h>
#include <torch/torch.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <opencv2/tracking.hpp>
#include <chrono>

using namespace std;
using namespace chrono;

class DigitalRecognition
{
private:
    torch::jit::script::Module module;
    torch::Device device;
    const int IMAGE_COLS = 28;
    const int IMAGE_ROWS = 28;

public:
    
    explicit DigitalRecognition(bool use_cuda = true,
                                const std::string &model_path = "./model/model.pt") : device(torch::kCPU)
    {
        if ((use_cuda) && (torch::cuda::is_available()))
        {
            // std::cout << "CUDA is available! Training on GPU." << std::endl;
            device = torch::kCUDA;
        }
        module = torch::jit::load(model_path, device);
    }
    int matToDigital(cv::Mat &img);
};



