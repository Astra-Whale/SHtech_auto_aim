//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//Std
#include <vector>
#include <fstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "signal.h"
#include <thread>
#include "pthread.h"
#include <dirent.h>
#include <mutex>
#include <atomic>
#include <condition_variable>

//Task
#include "detect.hpp"
#include "sensor.hpp"
#include "predict.hpp"

//Common
#include "common.hpp"

#define GPU
