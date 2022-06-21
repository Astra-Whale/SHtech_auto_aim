//
// Inherit from auto-aim/others/hikcam_warpper/warpper_head.hpp commit 58e05e7e Guanqi He on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Capture image from input sources
//

#ifndef CAM_CAM_WRAPPER_H
#define CAM_CAM_WRAPPER_H

//packages
#include <opencv2/core/core.hpp>

/**
 * @brief A virtual class for wrapper of camera and video files
 */
class WrapperHead
{
public:
    virtual ~WrapperHead() = default;
    ;
    virtual bool init() = 0;
    virtual bool read(cv::Mat &src) = 0;
    virtual bool setBrightness(int brightness) = 0;
    virtual int getFps() = 0;
    virtual cv::Size getSize() = 0;
    virtual bool close() = 0;
};

//submodules
#include <video/video_wrapper.hpp>
#include <hikcam/hikcam_wrapper.hpp>
#endif //CAM_CAM_WRAPPER_H
