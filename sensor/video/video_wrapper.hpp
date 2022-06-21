//
// Inherit from auto-aim commit 58e05e7e Guanqi He on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Refact framework.
// Warp image from video files
//

#ifndef CAM_VIDEO_WARPPER_H
#define CAM_VIDEO_WARPPER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cam_wrapper.hpp"

class VideoWrapper : public WrapperHead
{
public:
    VideoWrapper(const std::string &filename);
    ~VideoWrapper();

    /**
     * @brief initialize cameras
     * @return bool value: whether it success
     */
    bool init() final;

    /**
     * @brief read images from camera
     * @param src_left : output source video of left camera
     * @param src_right : output source video of right camera
     * @return bool value: whether the reading is successful
     */
    bool read(cv::Mat &src) final;
    bool setBrightness(int gain);
    int getFps();
    bool close();
    cv::Size getSize() { return cv::Size(video.get(cv::CAP_PROP_FRAME_WIDTH), video.get(cv::CAP_PROP_FRAME_HEIGHT)); };

private:
    cv::VideoCapture video;
};

#endif //CAM_VIDEO_WARPPER_H
