#ifndef HKCAM
#define HKCAM

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <pthread.h>
#include "MvCameraControl.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iostream>


namespace hnurm
{
    class HKcam
    {

    public:

        /**
         * @brief 
         * 
         */
        HKcam()
        {
            _cfg_path = "../CameraParam.yaml";
            _id = "";
        }

        HKcam(const cv::FileNode &cfg_node);

        ~HKcam();


        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool OpenCam(const std::string &cameraId = "");


        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool CloseCam();


        /**
         * @brief 
         * 
         */
//        bool SendFrame(cv::Mat& img);
        bool SendFrame(cv::Mat& img);

        /**
         * @brief Set params for camera
         *
         */
        void SetParam(const std::string &cfg_path);

        std::string GetCamName();

        bool SendFromVideoCapture(cv::Mat &img);

    private:

        //state num
        int nRet; 

        //handle for manipulating the Camera
        void* handle;
        
        //camera param
        MVCC_INTVALUE stParam{};

        //frame ptr
        unsigned char *pData;

        //format of frame ,read from camera
        MV_FRAME_OUT_INFO_EX stImageInfo{};

        //indicating whether the camera is connected or not
        //a flag for daemon thread
//        bool connected_flag;

        std::string _cfg_path  = "none";
        std::string _id = "none";
        std::string _video_path = "none";

        int _nImageOrientation = 0;

        cv::VideoCapture video;

    }; // HKcam

} // hnurm
#endif
