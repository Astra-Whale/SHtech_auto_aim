#include "cam_wrapper.hpp"
#include <iostream>

HikCamWrapper::HikCamWrapper(int dev_num)
{
    this->cam_handle = nullptr;
    this->exposure = exposure;
    this->gain = gain;
    this->pData = nullptr;
    this->nDataSize = 0;
    this->dev_num = dev_num;

    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT_INFO_EX));
}

bool HikCamWrapper::init()
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
            std::cout << "Enum Devices Fail!" << std::endl;
            break;
        }
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            std::cout << "[device " << i << "]:\n"
                      << std::endl;
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
            }
            PrintDeviceInfo(pDeviceInfo);
        }
        if (stDeviceList.nDeviceNum == 0)
        {
            std::cout << "Find No Devices!" << std::endl;
            break;
        }
        if (dev_num >= stDeviceList.nDeviceNum)
        {
            std::cout << "Out Of Device Index" << std::endl;
            break;
        }

        std::cout << "Camera index: " << dev_num << std::endl;

        // 选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&cam_handle, stDeviceList.pDeviceInfo[dev_num]);

        if (MV_OK != nRet)
        {
            std::cout << "MV_CC_CreateHandle Fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(cam_handle);
        if (MV_OK != nRet)
        {
            std::cout << "MV_CC_OpenDevice Fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // 设置触发模式为off
        nRet = MV_CC_SetEnumValue(cam_handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            std::cout << "MV_CC_SetTriggerMode Fail! nRet " << std::hex << nRet << std::endl;
            break;
        }

        // ch:获取数据包大小 | en:Get payload size
        nRet = MV_CC_GetIntValue(cam_handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            std::cout << "Get PayloadSize fail! nRet " << std::hex << nRet << std::endl;
        }

        // 开始取流
        nRet = MV_CC_StartGrabbing(cam_handle);
        if (MV_OK != nRet)
        {
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
                std::cout << "Fail To Read First 10 Frames" << std::endl;
            }
            if (NULL != stOutFrame.pBufAddr)
            {
                nRet = MV_CC_FreeImageBuffer(cam_handle, &stOutFrame);
                if (nRet != MV_OK)
                {
                    std::cout << "Free Image Buffer fail! nRet " << std::hex << nRet << std::endl;
                    break;
                }
            }
        }
    } while (0);

    if (nRet != MV_OK)
    {
        close();
        return false;
    }

    return true;
}

bool HikCamWrapper::close()
{
    std::cout << "closing camera...\n";
    int nRet = MV_OK;

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(cam_handle);
    if (MV_OK != nRet)
    {
        std::cout << "MV_CC_StopGrabbing fail! nRet " << nRet << std::endl;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(cam_handle);
    if (MV_OK != nRet)
    {
        std::cout << "MV_CC_CloseDevice fail! nRet " << nRet << std::endl;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(cam_handle);
    if (MV_OK != nRet)
    {
        std::cout << "MV_CC_DestroyHandle fail! nRet " << nRet << std::endl;
    }
    if (!pData)
    {
        free(pData);
    }
    std::cout << "HikCam Close Success" << std::endl;
    return nRet == MV_OK;
}

HikCamWrapper::~HikCamWrapper()
{
    close();
}

bool HikCamWrapper::read(cv::Mat &src)
{
    int nRet = MV_CC_GetImageBuffer(cam_handle, &stOutFrame, 1000);
    //int nRet = MV_CC_GetOneFrameTimeout(cam_handle, pData, nDataSize, &stImageInfo, 20);

    if (MV_OK != nRet)
    {
        std::cout << "MV_CC_GetImageBuffer fail! nRet" << nRet << std::endl;
        return false;
    }

    //nRet = MV_CC_ConvertPixelType(cam_handle, &stConvertParam);
    //src = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);
    cv::Mat _src;

    _src = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 0, stOutFrame.pBufAddr); // Mat date type CV_8UC = 0

    cv::cvtColor(_src, src, cv::COLOR_BayerRG2RGB); // bayer RG -> BGR
    
    if (NULL != stOutFrame.pBufAddr)
    {
        nRet = MV_CC_FreeImageBuffer(cam_handle, &stOutFrame);
        if (nRet != MV_OK)
        {
            std::cout << "Free Image Buffer fail! nRet " << std::hex << nRet << std::endl;
            return false;
        }
    }

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
    MV_CC_GetImageInfo(cam_handle, &stCamInfo);
    return stCamInfo.fFrameRateValue;
}

cv::Size HikCamWrapper::getSize()
{
    MV_CC_GetImageInfo(cam_handle, &stCamInfo);
    return cv::Size(stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight);
}

bool HikCamWrapper::setBrightness(int brightness)
{
    return MV_CC_SetBrightness(cam_handle, brightness) == MV_OK;
}
