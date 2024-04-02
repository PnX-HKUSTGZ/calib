#include "Camera.h"
#include <chrono>
#include <cstdio>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/operations.hpp>
#include <string>

using namespace cv;
using namespace std;
namespace hnurm
{
    HKcam::HKcam(const cv::FileNode &cfg_node)
    {
        nRet = MV_OK;
        handle = nullptr;
        pData = nullptr;

        cfg_node["camera_id"] >> _id;
        cfg_node["camera_param_path"] >> _cfg_path;
        cfg_node["image_orientation"] >> _nImageOrientation;
        cfg_node["video_path"] >> _video_path;


        //没准备视频则打开相机
        if (!this->video.isOpened()) {
            if (OpenCam(_id))// here open and config camera
            {
                std::cout << "Camera " << _id << " opened successfully.";
            } else
            {
                std::cout << "Camera " << _id << " failed to open!";
            }


        }
    }

    HKcam::~HKcam()
    {
        if (!this->video.isOpened()) {
            CloseCam();
        }
    }

    void HKcam::SetParam(const std::string &cfg_path)
    {
        //Get the setParam file
        FileStorage CameraParam(cfg_path, FileStorage::READ);
        if (!CameraParam.isOpened())
        {
            std::cerr << "failed to open CameraParam.xml" << std::endl;
            return;
        }

        // 宽设置时需考虑步进(16)，即设置宽需16的倍数
        int nWidthValue;
        CameraParam["nWidthValue"] >> nWidthValue;
        nRet = MV_CC_SetIntValue(handle, "Width", nWidthValue);
        if (MV_OK == nRet)
            printf("set Width = %d OK!\n", nWidthValue);
        else
            printf("set Width failed! nRet [%x]\n", nRet);

        // 高设置时需考虑步进(2)，即设置高需16的倍数
        int nHeightValue;
        CameraParam["nHeightValue"] >> nHeightValue;
        nRet = MV_CC_SetIntValue(handle, "Height", nHeightValue);
        if (MV_OK == nRet)
            printf("set height = %d OK!\n", nHeightValue);
        else
            printf("set height failed! nRet [%x]\n", nRet);

        // 设置水平偏移
        int nOffsetXValue;
        CameraParam["nOffsetXValue"] >> nOffsetXValue;
        nRet = MV_CC_SetIntValue(handle, "OffsetX", nOffsetXValue);
        if (MV_OK == nRet)
            printf("set OffsetX = %d OK!\n", nOffsetXValue);
        else
            printf("set OffsetX failed! nRet [%x]\n", nRet);

        // 设置垂直偏移
        int nOffsetYValue;
        CameraParam["nOffsetYValue"] >> nOffsetYValue;
        nRet = MV_CC_SetIntValue(handle, "OffsetY", nOffsetYValue);
        if (MV_OK == nRet)
            printf("set OffsetY = %d OK!\n", nOffsetYValue);
        else
            printf("set OffsetY failed! nRet [%x]\n", nRet);

        // 设置水平镜像
        bool bSetBoolValue5;
        CameraParam["bSetBoolValue5"] >> bSetBoolValue5;
        nRet = MV_CC_SetBoolValue(handle, "ReverseX", bSetBoolValue5);
        if (MV_OK == nRet)
            printf("set ReverseX = %d OK!\n", bSetBoolValue5);
        else
            printf("set ReverseX Failed! nRet = [%x]\n", nRet);

        // 设置垂直镜像
        bool bSetBoolValue1;
        CameraParam["bSetBoolValue1"] >> bSetBoolValue1;
        nRet = MV_CC_SetBoolValue(handle, "ReverseY", bSetBoolValue1);
        if (MV_OK == nRet)
            printf("Set ReverseY = %d OK!\n", bSetBoolValue1);
        else
            printf("Set ReverseY Failed! nRet = [%x]\n", nRet);

        // 设置像素格式
        int nPixelFormat;
        CameraParam["PixelFormat"] >> nPixelFormat;
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", nPixelFormat);
        if (MV_OK == nRet)
            printf("set PixelFormat = %x OK!\n", nPixelFormat);
        else
            printf("set PixelFormat failed! nRet [%x]\n", nRet);

        // 设置采集触发帧率
        int nAcquisitionBurstFrameCountValue;
        CameraParam["nAcquisitionBurstFrameCountValue"] >> nAcquisitionBurstFrameCountValue;
        nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount",
                                 nAcquisitionBurstFrameCountValue);
        if (MV_OK == nRet)
            printf("set AcquisitionBurstFrameCount = %d OK!\n", nAcquisitionBurstFrameCountValue);
        else
            printf("set AcquisitionBurstFrameCount failed! nRet [%x]\n", nRet);

        // 设置采集帧率
        float fFPSValue;
        CameraParam["fFPSValue"] >> fFPSValue;
        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fFPSValue);

        if (MV_OK == nRet)
            printf("set AcquisitionFrameRate = %f OK!\n", fFPSValue);
        else
            printf("set AcquisitionFrameRate failed! nRet [%x]\n", nRet);

        // 设置使能采集帧率控制
        bool bSetBoolValue3;
        CameraParam["bSetBoolValue3"] >> bSetBoolValue3;
        nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", bSetBoolValue3);
        if (MV_OK == nRet)
            printf("Set AcquisitionFrameRateEnable = %d OK!\n", bSetBoolValue3);
        else
            printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n", nRet);

        // 设置曝光时间
        float fExposureTime;
        CameraParam["fExposureTime"] >> fExposureTime;
        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", fExposureTime);
        if (MV_OK == nRet)
            printf("set ExposureTime = %f OK!\n", fExposureTime);
        else
            printf("set ExposureTime failed! nRet [%x]\n", nRet);

        // 设置增益
        float fGainValue;
        CameraParam["fGainValue"] >> fGainValue;
        nRet = MV_CC_SetFloatValue(handle, "Gain", fGainValue);
        if (MV_OK == nRet)
            printf("set Gain = %f OK!\n", fGainValue);
        else
            printf("set Gain failed! nRet [%x]\n", nRet);

        // 设置黑电平
        int nBlackLevelValue;
        CameraParam["nBlackLevelValue"] >> nBlackLevelValue;
        nRet = MV_CC_SetIntValue(handle, "BlackLevel", nBlackLevelValue);
        if (MV_OK == nRet)
            printf("set BlackLevel = %d OK!\n", nBlackLevelValue);
        else
            printf("set BlackLevel failed! nRet [%x]\n", nRet);

        // 设置黑电平使能
        bool bSetBoolValue2;
        CameraParam["bSetBoolValue2"] >> bSetBoolValue2;
        nRet = MV_CC_SetBoolValue(handle, "BlackLevelEnable", bSetBoolValue2);
        if (MV_OK == nRet)
            printf("Set BlackLevelEnable = %d OK!\n", bSetBoolValue2);
        else
            printf("Set BlackLevelEnable Failed! nRet = [%x]\n", nRet);
    }

    bool HKcam::OpenCam(const string &cameraID)
    {
        //        nRet = MV_OK;
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (!stDeviceList.nDeviceNum)
        {
            printf("Find No Devices! nRet = [%x]\n", nRet);
            return false;
        }

        //select the first camera connected
        unsigned int nIndex = 0;

        while (true)
        {
            // 选择设备并创建句柄
            // select device and create handle
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
                return false;
            }

            // 获取设备id
            // get device id
            stringstream ss;
            ss << stDeviceList.pDeviceInfo[nIndex]->SpecialInfo.stUsb3VInfo.chDeviceGUID;
            ss >> _id;
            cout << "camera id " << _id << endl;

            // 若指定了相机id，则判断是否为指定相机
            // if camera id is specified, check if it is the specified camera
            if (!cameraID.empty())
            {
                if (cameraID != _id)// 若不是指定相机，则关闭句柄并继续枚举
                {
                    printf("camera id %s not matched to desired %s\n", _id.c_str(), cameraID.c_str());
                    MV_CC_CloseDevice(handle);
                    MV_CC_DestroyHandle(handle);
                    nIndex++;
                    if (nIndex >= stDeviceList.nDeviceNum)// 若已枚举完所有相机，则返回
                    {
                        printf("Find No Devices!\n");
                        return false;
                    }
                    continue;
                } else
                {
                    printf("ready to open camera %s\n", _id.c_str());
                }
            } else
            {
                printf("camera id not set, ready to open camera %s\n", _id.c_str());
            }

            // 打开设备
            // open device
            nRet = MV_CC_OpenDevice(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
                return false;
            }

            // 设置触发模式为off
            // set trigger mode as off
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
                return false;
            }

            // set param
            // 设置参数
            SetParam(_cfg_path);

            // Get payload size
            memset(&stParam, 0, sizeof(MVCC_INTVALUE));
            nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
            if (MV_OK != nRet)
            {
                printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
                return false;
            }

            //s tart grab stream
            nRet = MV_CC_StartGrabbing(handle);
            if (MV_OK != nRet)
            {
                printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
                return false;
            }

            // check
            stImageInfo = {0};
            memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
            pData = (unsigned char *) malloc(sizeof(unsigned char) * stParam.nCurValue);
            if (NULL == pData)
            {
                std::cout << "can't get size of a frame!" << std::endl;
                return false;
            }

            return true;
        }
    }

    bool HKcam::CloseCam()
    {
        try {
            // 停止取流
            // end grab image
            nRet = MV_CC_StopGrabbing(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
                return false;
            }
            // 关闭设备
            // close device
            nRet = MV_CC_CloseDevice(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
                return false;
            }
            // 销毁句柄
            // destroy handle
            nRet = MV_CC_DestroyHandle(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
                return false;
            }
            return true;
        } catch (...)
        {
            printf("something went wrong...");
            return false;
        }
    }

    bool HKcam::SendFrame(cv::Mat &img)
    {
        nRet = MV_OK;
        // todo get time stamp for imu alignment
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, stParam.nCurValue, &stImageInfo, 10000);

        if (nRet != MV_OK)
        {
            printf("No data[0x%x]\n", nRet);
            return false;
        }

        img = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
        //        cvtColor(tmp_bayer, img, cv::COLOR_BayerGB2RGB);

        // 根据参数对img旋转
        switch (_nImageOrientation)
        {
            case 0:
                break;
            case 1:
                cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
                break;
            case 2:
                cv::rotate(img, img, cv::ROTATE_180);
                break;
            case 3:
                cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);
                break;
        }
        return true;
    }

    string HKcam::GetCamName()
    {
        return _id;
    }

    //循环播放视频
    bool HKcam::SendFromVideoCapture(cv::Mat &img)
    {
        bool ret = this->video.isOpened();
        if (ret){
            static int count = 0;
            this->video >> img;
            count++;
            if (count == this->video.get(cv::CAP_PROP_FRAME_COUNT)){
                count = 0 ;
                this->video.set(cv::CAP_PROP_POS_FRAMES,0);
            }
        }
        return ret;
    }

}// namespace hnurm
