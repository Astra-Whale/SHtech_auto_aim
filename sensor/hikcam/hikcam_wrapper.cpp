#include "cam_wrapper.hpp"
#include "hikcam_wrapper.hpp"
#include <iostream>

namespace
{
const char *BayerPatternToString(HikCamWrapper::BayerPattern pattern)
{
    switch (pattern)
    {
    case HikCamWrapper::BayerPattern::RG:
        return "BayerRG";
    case HikCamWrapper::BayerPattern::GB:
        return "BayerGB";
    case HikCamWrapper::BayerPattern::AUTO:
    default:
        return "AUTO";
    }
}

int PixelTypeToCvType(unsigned int pixel_type)
{
    switch (pixel_type)
    {
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
        return CV_8UC1;
    case PixelType_Gvsp_BayerRG10:
    case PixelType_Gvsp_BayerRG12:
    case PixelType_Gvsp_BayerRG16:
    case PixelType_Gvsp_BayerGB10:
    case PixelType_Gvsp_BayerGB12:
    case PixelType_Gvsp_BayerGB16:
        return CV_16UC1;
    default:
        return -1;
    }
}
}

HikCamWrapper::HikCamWrapper(int dev_num, BayerPattern bayer_pattern)
    : exposure(0.0),
      gain(0.0),
      cam_handle(nullptr),
      dev_num(dev_num),
      fail_cnt(0),
      pData(nullptr),
      pDataForBGR(nullptr),
      nDataSize(0),
      configured_bayer_pattern_(bayer_pattern),
      bayer_pattern_(bayer_pattern)
{
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
}

bool HikCamWrapper::init(bool debug)
{
    int nRet = MV_OK;
    do
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        //枚举设备
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            if (debug)
                std::cout << "Enum Devices Fail!" << std::endl;
            break;
        }
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                continue;
            }
            if (debug)
            {
                std::cout << "[device " << i << "]:\n"
                          << std::endl;
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        if (stDeviceList.nDeviceNum == 0)
        {
            if (debug)
                std::cout << "Find No Devices!" << std::endl;
            break;
        }
        if (dev_num >= stDeviceList.nDeviceNum)
        {
            if (debug)
                std::cout << "Out Of Device Index" << std::endl;
            break;
        }
        
        if (debug)
            std::cout << "Camera index: " << dev_num << std::endl;

        // 选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&cam_handle, stDeviceList.pDeviceInfo[dev_num]);

        if (MV_OK != nRet)
        {
            if (debug)
                std::cout << "MV_CC_CreateHandle Fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(cam_handle);
        if (MV_OK != nRet)
        {
            if (debug)
                std::cout << "MV_CC_OpenDevice Fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // 设置触发模式为off
        if (configured_bayer_pattern_ == BayerPattern::AUTO)
        {
            MVCC_ENUMVALUE stPixelFormat;
            memset(&stPixelFormat, 0, sizeof(MVCC_ENUMVALUE));
            nRet = MV_CC_GetPixelFormat(cam_handle, &stPixelFormat);
            if (MV_OK != nRet)
            {
                if (debug)
                    std::cout << "MV_CC_GetPixelFormat Fail! nRet " << std::hex << nRet << std::endl;
                break;
            }

            if (!updateBayerPatternFromPixelType(stPixelFormat.nCurValue))
            {
                if (debug)
                    std::cout << "Unsupported pixel format for HikCamWrapper: 0x"
                              << std::hex << stPixelFormat.nCurValue << std::dec << std::endl;
                break;
            }

            if (debug)
                std::cout << "Detected Bayer pattern: " << BayerPatternToString(bayer_pattern_) << std::endl;
        }
        else
        {
            bayer_pattern_ = configured_bayer_pattern_;
        }

        nRet = MV_CC_SetEnumValue(cam_handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            if (debug)
                std::cout << "MV_CC_SetTriggerMode Fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // ch:获取数据包大小 | en:Get payload size
        nRet = MV_CC_GetIntValue(cam_handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            if (debug)
                std::cout << "Get PayloadSize fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // 开始取流
        nRet = MV_CC_StartGrabbing(cam_handle);
        if (MV_OK != nRet)
        {
            if (debug)
                std::cout << "MV_CC_StartGarbbing Fail! nRet" << std::hex << nRet << std::endl;
            break;
        }

        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);

        nDataSize = stParam.nCurValue;

        for (int i = 0; i < 10; i++)
        {
            nRet = MV_CC_GetImageBuffer(cam_handle, &stOutFrame, 1000);
            if (MV_OK != nRet)
            {
                if (debug)
                    std::cout << "Fail To Read First 10 Frames" << std::endl;
            }
            if (NULL != stOutFrame.pBufAddr)
            {
                nRet = MV_CC_FreeImageBuffer(cam_handle, &stOutFrame);
                if (nRet != MV_OK)
                {
                    if (debug)
                        std::cout << "Free Image Buffer fail! nRet " << std::hex << nRet << std::endl;
                    break;
                }
            }
        }
    } while (0);

    if (nRet != MV_OK)
    {
        close(debug);
        return false;
    }
    fail_cnt = 0;
    return true;
}

bool HikCamWrapper::close(bool debug)
{
    if (debug)
        std::cout << "closing camera...\n";
    int nRet = MV_OK;

    if (cam_handle == nullptr)
    {
        if (pData)
        {
            free(pData);
            pData = nullptr;
        }
        if (debug)
            std::cout << "HikCam handle is null, skip SDK close calls" << std::endl;
        return true;
    }

    if (NULL != stOutFrame.pBufAddr)
    {
        int freeRet = MV_CC_FreeImageBuffer(cam_handle, &stOutFrame);
        if (freeRet != MV_OK && debug)
        {
            std::cout << "Free Image Buffer before close fail! nRet " << std::hex << freeRet << std::endl;
        }
        memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    }

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(cam_handle);
    if (MV_OK != nRet)
    {
        if (debug)
            std::cout << "MV_CC_StopGrabbing fail! nRet " << nRet << std::endl;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(cam_handle);
    if (MV_OK != nRet)
    {
        if (debug)
            std::cout << "MV_CC_CloseDevice fail! nRet " << nRet << std::endl;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(cam_handle);
    if (MV_OK != nRet)
    {
        if (debug)
            std::cout << "MV_CC_DestroyHandle fail! nRet " << nRet << std::endl;
    }
    cam_handle = nullptr;
    if (pData)
    {
        free(pData);
        pData = nullptr;
    }
    if (debug)
        std::cout << "HikCam Close Success" << std::endl;
    return nRet == MV_OK;
}

HikCamWrapper::~HikCamWrapper()
{
    close();
}

bool HikCamWrapper::read(cv::Mat &src, bool debug)
{
    if (fail_cnt > 10)
    {
        close(debug);
        if (!init(debug))
        {
            return false;
        }
    }

    int nRet = MV_CC_GetImageBuffer(cam_handle, &stOutFrame, 1000);
    //int nRet = MV_CC_GetOneFrameTimeout(cam_handle, pData, nDataSize, &stImageInfo, 20);

    if (MV_OK != nRet)
    {
        if (debug)
            std::cout << "MV_CC_GetImageBuffer fail! nRet" << nRet << std::endl;
        fail_cnt++;
        return false;
    }

    //nRet = MV_CC_ConvertPixelType(cam_handle, &stConvertParam);
    //src = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);
    cv::Mat _src;
    const unsigned int pixel_type = stOutFrame.stFrameInfo.enPixelType;
    const int cv_type = PixelTypeToCvType(pixel_type);

    if (cv_type < 0)
    {
        static bool unexpected_pixel_type_logged = false;
        if (!unexpected_pixel_type_logged)
        {
            std::cerr << "[HikCamWrapper] Reject unsupported pixel type: 0x"
                      << std::hex << pixel_type << std::dec << std::endl;
            unexpected_pixel_type_logged = true;
        }

        if (NULL != stOutFrame.pBufAddr)
        {
            int freeRet = MV_CC_FreeImageBuffer(cam_handle, &stOutFrame);
            if (freeRet != MV_OK)
            {
                if (debug)
                    std::cout << "Free Image Buffer fail! nRet " << std::hex << freeRet << std::endl;
                fail_cnt++;
                return false;
            }
            memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
        }

        fail_cnt++;
        return false;
    }

    _src = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, cv_type, stOutFrame.pBufAddr);

    cv::cvtColor(_src, src, getCvBayerConversionCode());
    
    if (NULL != stOutFrame.pBufAddr)
    {
        nRet = MV_CC_FreeImageBuffer(cam_handle, &stOutFrame);
        if (nRet != MV_OK)
        {
            if (debug)
                std::cout << "Free Image Buffer fail! nRet " << std::hex << nRet << std::endl;
            fail_cnt++;
            return false;
        }
        memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    }
    fail_cnt = 0;
    return true;
}

bool HikCamWrapper::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

int HikCamWrapper::getFps()
{
    if (cam_handle == nullptr)
    {
        std::cerr << "[HikCamWrapper] getFps called with null camera handle" << std::endl;
        return 0;
    }

    MV_CC_GetImageInfo(cam_handle, &stCamInfo);
    return stCamInfo.fFrameRateValue;
}

cv::Size HikCamWrapper::getSize()
{
    if (cam_handle == nullptr)
    {
        std::cerr << "[HikCamWrapper] getSize called with null camera handle" << std::endl;
        return cv::Size(0, 0);
    }

    MV_CC_GetImageInfo(cam_handle, &stCamInfo);
    return cv::Size(stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight);
}

bool HikCamWrapper::setBrightness(int brightness)
{
    if (cam_handle == nullptr)
    {
        std::cerr << "[HikCamWrapper] setBrightness called with null camera handle" << std::endl;
        return false;
    }

    return MV_CC_SetBrightness(cam_handle, brightness) == MV_OK;
}

void HikCamWrapper::setBayerPattern(BayerPattern bayer_pattern)
{
    configured_bayer_pattern_ = bayer_pattern;
    bayer_pattern_ = bayer_pattern;
}

bool HikCamWrapper::updateBayerPatternFromPixelType(unsigned int pixel_type)
{
    switch (pixel_type)
    {
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerRG10:
    case PixelType_Gvsp_BayerRG12:
    case PixelType_Gvsp_BayerRG16:
        bayer_pattern_ = BayerPattern::RG;
        return true;
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerGB10:
    case PixelType_Gvsp_BayerGB12:
    case PixelType_Gvsp_BayerGB16:
        bayer_pattern_ = BayerPattern::GB;
        return true;
    default:
        return false;
    }
}

int HikCamWrapper::getCvBayerConversionCode() const
{
    switch (bayer_pattern_)
    {
    case BayerPattern::RG:
        return cv::COLOR_BayerRG2RGB;
    case BayerPattern::GB:
        return cv::COLOR_BayerGB2RGB;
    case BayerPattern::AUTO:
    default:
        return cv::COLOR_BayerRG2RGB;
    }
}
