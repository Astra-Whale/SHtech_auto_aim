#include "MvCameraControl.h"
#include "pthread.h"
#include <opencv2/opencv.hpp>
#include "../cam_wrapper.hpp"

class HikCamWrapper : public WrapperHead
{
public:
    enum class BayerPattern
    {
        AUTO = 0,
        RG,
        GB
    };

    cv::Mat img;

    HikCamWrapper(int dev_num = 0, BayerPattern bayer_pattern = BayerPattern::AUTO);
    ~HikCamWrapper();

    bool init(bool debug = false);

    bool setBrightness(int brightness);
    int getFps();
    void setBayerPattern(BayerPattern bayer_pattern);

    bool read(cv::Mat &src, bool debug = false);

    bool close(bool debug = false);

    cv::Size getSize();

private:
    double exposure;
    double gain;
    void *cam_handle;
    unsigned int dev_num;
    unsigned int fail_cnt;

    unsigned char *pData;
    unsigned char *pDataForBGR;
    int nDataSize;
    BayerPattern configured_bayer_pattern_;
    BayerPattern bayer_pattern_;

    MV_IMAGE_BASIC_INFO stCamInfo;
    MV_FRAME_OUT stOutFrame = {0};
    MVCC_INTVALUE stParam;

    bool updateBayerPatternFromPixelType(unsigned int pixel_type);
    int getCvBayerConversionCode() const;
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
};
