//
// Inherit from auto-aim commit 58e05e7e Guanqi He on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Warp image from video files
//

#include "cam_wrapper.hpp"
#include "video_wrapper.hpp"
#include <iostream>

VideoWrapper::VideoWrapper(const std::string &filename)
{
    video.open(filename);
}

VideoWrapper::~VideoWrapper() {}

bool VideoWrapper::init(bool debug)
{
    return video.isOpened();
}

bool VideoWrapper::close(bool debug) { return false; };

bool VideoWrapper::read(cv::Mat &src, bool debug)
{
    return video.read(src);
}

bool VideoWrapper::setBrightness(int brightness)
{
    std::cerr << "[VideoWrapper] setBrightness is not supported for file input" << std::endl;
    return false;
}

int VideoWrapper::getFps()
{
    return video.get(5);
}
