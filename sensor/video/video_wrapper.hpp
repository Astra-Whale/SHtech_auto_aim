//
// Inherit from auto-aim commit 58e05e7e Guanqi He on 21-05-24.
// Modified by Haoran Jiang on 21-10-02: Updated the framework.
// Read images from video files.
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
     * @brief initialize the video file
     * @return `true` when the video file is open
     */
    bool init(bool debug = false) final;

    /**
     * @brief read the next video frame
     * @param src output video frame
     * @return `true` when a frame is read successfully
     */
    bool read(cv::Mat &src, bool debug = false) final;
    bool setBrightness(int gain);
    int getFps();
    bool close(bool debug = false);
    cv::Size getSize() { return cv::Size(video.get(cv::CAP_PROP_FRAME_WIDTH), video.get(cv::CAP_PROP_FRAME_HEIGHT)); };

private:
    cv::VideoCapture video;
};

#endif //CAM_VIDEO_WARPPER_H
