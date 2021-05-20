#ifndef _DERECROR_HPP_
#define _DERECROR_HPP_
//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core_c.h>
//Std
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
//YOLO
#include "yolo_v2_class.hpp"
//Config
#include "config.h"
#include "image.hpp"

class ArmorDetector
{
public:
    ArmorDetector(const std::string &classesFile, const std::string &modelConfig, const std::string &modelWeights, bool enemy);
    bool detect(std::shared_ptr<image_t> detImg, const Image &frame, std::vector<bbox_t> &outs);
    ~ArmorDetector();

    Detector *detector;

    //Draw bbox
    void Drawer(cv::Mat &frame, const std::vector<bbox_t> &outs, const std::vector<std::string> &classes);

private:
    bool enemy;
    std::string enemy_label;

    std::string classesFile;
    std::string modelConfig;
    std::string modelWeights;
    std::vector<std::string> classes;

    void DrawBoxes(cv::Mat &frame, std::vector<std::string> classes, int classId, float conf, int left, int top, int right, int bottom);
};

#endif