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
#include <cstdio>
#include <string>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <thread>
#include "pthread.h"
#include <dirent.h>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <execinfo.h>
#include <future>
#include <map>

//Submodules
#include "entrystage/entryStage_submodule.hpp"
#include "sensor/sensor_submodule.hpp"
#include "timedserial/timed_serial.hpp"
#include "timedserial/serial_interface.hpp"
#include "timedserial/UartIMU/uart_driver.hpp"
#include "timedserial/UartIMU/mock_driver.hpp"
#include "detect/preprocess_submodule.hpp"
#include "detect/corner_refine_submodule.hpp"
#include "detect/detect_submodule.hpp"
#include "predict/MultiPolicyPredictor_submodule.hpp"
#ifdef ENABLE_FOXGLOVE
#include "foxglove/foxglove_server.hpp"
#endif
#include "planner/planner_submodule.hpp"

//Math Utils
#include "mathutils/CoordTransformer.hpp"

//Common
#include "common.hpp"

#define GPU

using pipeline::autoaim_pipeline;
