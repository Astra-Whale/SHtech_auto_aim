#include "MvCameraControl.h"
#include "pthread.h"
#include <opencv2/opencv.hpp>
#include "../cam_wrapper.hpp"

class HikCamWrapper : public WrapperHead
{
public:
    cv::Mat img;

    HikCamWrapper(int dev_num = 0);
    ~HikCamWrapper();

    bool init();

    bool setBrightness(int brightness);
    int getFps();

    bool read(cv::Mat &src);

    bool close();

    cv::Size getSize();

private:
    double exposure;
    double gain;
    void *cam_handle;
    unsigned int dev_num;

    unsigned char *pData;
    unsigned char *pDataForBGR;
    int nDataSize;

    MV_IMAGE_BASIC_INFO stCamInfo;
    MV_FRAME_OUT stOutFrame = {0};
    MVCC_INTVALUE stParam;

    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
};
