#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__
#include <opencv2/opencv.hpp>

typedef struct 
{
    cv::Mat frame;
    unsigned int index;
}Image;


#endif
