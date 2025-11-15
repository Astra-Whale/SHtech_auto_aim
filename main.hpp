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

//Submodules
#include "entrystage/entryStage_submodule.hpp"
#include "sensor/sensor_submodule.hpp"
#include "cboard/cboard_submodule.hpp"
#include "detect/detect_submodule.hpp"
#include "predict/linearPredictor_submodule.hpp"

//Common
#include "common.hpp"

#define GPU

using pipeline::autoaim_pipeline;