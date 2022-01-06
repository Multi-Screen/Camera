﻿#include <iostream>
#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include <conio.h>
#include <string.h>
#include <Windows.h>
#include "opencv2/aruco.hpp"
#include <mutex>
#include <condition_variable>

#include<winsock.h>
#pragma comment(lib,"ws2_32.lib")

std::mutex data_mutex;
std::condition_variable data_var;
int flag = 1;


using namespace std;
using namespace cv;
cv::Mat src_img;

// 标签大小
float labelWidth = 0.0167;

unsigned int payload_size = 0;
bool g_bExit = false;

int ret = MV_OK;

// ======================socket========================
int send_len = 0;
int recv_len = 0;
//定义发送缓冲区和接受缓冲区
char send_buf[100];
char recv_buf[100];
//定义服务端套接字，接受请求套接字
SOCKET s_server;
//服务端地址客户端地址
SOCKADDR_IN server_addr;

/*socket初始化，服务器连接*/
void initialization() {
    //初始化套接字库
    WORD w_req = MAKEWORD(2, 2);//版本号
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        cout << "初始化套接字库失败！" << endl;
    }
    else {
        cout << "初始化套接字库成功！" << endl;
    }
    //检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        cout << "套接字库版本号不符！" << endl;
        WSACleanup();
    }
    else {
        cout << "套接字库版本正确！" << endl;
    }
    //填充服务端地址信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.1.123");
    server_addr.sin_port = htons(2020);
    //创建套接字
    s_server = socket(AF_INET, SOCK_STREAM, 0);
    cout << "正在连接服务器..." << endl;
    if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        cout << "服务器连接失败！" << endl;
        WSACleanup();
    }
    else {
        cout << "服务器连接成功！" << endl;
    }
}

// ch:等待按键输入 | en:Wait for key press
void WaitForKeyPress(void)
{
    while (!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}


bool printDeviceInfo(MV_CC_DEVICE_INFO* wt_device) {
    if (NULL == wt_device)
    {
        printf("The Pointer of wt_device is NULL!\n");
        return false;
    }
    if (wt_device->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((wt_device->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((wt_device->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((wt_device->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (wt_device->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", wt_device->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (wt_device->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", wt_device->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", wt_device->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", wt_device->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
    if (NULL == pRgbData)
    {
        return MV_E_PARAMETER;
    }

    for (unsigned int j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }
    return MV_OK;
}
 
// 3D角坐标 https://stackoverflow.com/questions/46363618/aruco-markers-with-opencv-get-the-3d-corner-coordinates
vector<Point3f> getCornersInCameraWorld(double side, Vec3d rvec, Vec3d tvec) {

    double half_side = side / 2;


    // compute rot_mat
    Mat rot_mat;
    Rodrigues(rvec, rot_mat);

    // transpose of rot_mat for easy columns extraction
    Mat rot_mat_t = rot_mat.t();

    // the two E-O and F-O vectors
    double* tmp = rot_mat_t.ptr<double>(0);
    Point3f camWorldE(tmp[0] * half_side,
        tmp[1] * half_side,
        tmp[2] * half_side);

    tmp = rot_mat_t.ptr<double>(1);
    Point3f camWorldF(tmp[0] * half_side,
        tmp[1] * half_side,
        tmp[2] * half_side);

    // convert tvec to point
    Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

    // return vector:
    vector<Point3f> ret(4, tvec_3f);

    ret[0] += camWorldE + camWorldF;
    ret[1] += -camWorldE + camWorldF;
    ret[2] += -camWorldE - camWorldF;
    ret[3] += camWorldE - camWorldF;

    return ret;
}

// convert data stream in Mat format
bool Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData, cv::Mat& src_img)
{
    cv::Mat srcImage;
    if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }
    else
    {
        printf("unsupported pixel format\n");
        return false;
    }

    if (NULL == srcImage.data)
    {
        return false;
    }

    //save converted image in a local file
    //try {
    //    cv::imwrite("MatImage.bmp", srcImage);
    //}
    //catch (cv::Exception& ex) {
    //    fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
    //}
    srcImage.copyTo(src_img);
    srcImage.release();
    return true;
}

static  unsigned int __stdcall  WorkThread(void* pUser)
{   
    MV_FRAME_OUT_INFO_EX wt_imginfo;
    memset(&wt_imginfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    //std::cout <<  std::this_thread::get_id() << "   printf: " << "WorkThread1" << std::endl;
    unsigned char* data = (unsigned char*)malloc(sizeof(unsigned char) * (payload_size));
    while (1) {
        std::unique_lock<std::mutex> lck(data_mutex);
        data_var.wait(lck, [] {return flag == 1; });
       
        if (data == NULL) {
            return -1;
        }
        ret = MV_CC_GetOneFrameTimeout(pUser, data, payload_size, &wt_imginfo, 1000);
        if (ret != MV_OK) {
            free(data);
            data = NULL;
            return -1;
        }
        if (Convert2Mat(&wt_imginfo, data, src_img) == false) {
            return -1;
        }
        //=======================
        if (g_bExit) {
            break;
        }

        flag = 2;
        data_var.notify_all();
    }
    return 0;
}

// 计算坐标--相对于摄像机，摄像机是世界坐标的0 0 0
float init_data[4] = { 0,0,0,1 };
Mat camera_pos = Mat(4, 1, CV_32F, &init_data);

static  unsigned int __stdcall  WorkThread2(void* pUser)
{
    while (1) {
        std::unique_lock<std::mutex> lck(data_mutex);
        data_var.wait(lck, [] {return flag == 2; });

        //std::cout << "thread: " << std::this_thread::get_id() << "   printf: " << "WorkThread2" << std::endl;
        //std::chrono::time_point<std::chrono::high_resolution_clock> p0 = std::chrono::high_resolution_clock::now();
        //========处理检测码============
         // load intrinsics,需要标定相机才能获取
        cv::Mat cameraMatrix = Mat(3, 3, CV_32F), distCoeffs = Mat(1, 5, CV_32F);
        Mat Rt = Mat(3, 4, CV_32F);
        vector<vector<float>> intrinsics = { 
            {3535.78,0,860.79} ,
            {0,3551.96,480.73},
            {0,0,1} 
        };
        vector<vector<float>> distCoeffsMat = { 
            {-0.0511285 ,1.47971 ,-0.00731253 ,0,0} 
        };
        for (int i = 0; i < cameraMatrix.rows; i++)
        {
            for (int j = 0; j < cameraMatrix.cols; j++)
            {
                cameraMatrix.at<float>(i, j) = intrinsics[i][j];
            }
        }

        for (int i = 0; i < distCoeffs.rows; i++)
        {
            for (int j = 0; j < distCoeffs.cols; j++)
            {
                distCoeffs.at<float>(i, j) = distCoeffsMat[i][j];
            }
        }
        
        Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_4X4_100));
        // board: aruco map. create(x_num, y_num, size(m), gap, diction, (index=1))
        Ptr<aruco::GridBoard> board = aruco::GridBoard::create(7, 10, labelWidth, 0.004, dict);   // real distance in meters.
        vector<int> markerIds;         // detected ids.
        vector<vector<Point2f>> markerCorners;
        aruco::detectMarkers(src_img, board->dictionary, markerCorners, markerIds);
        
        if (markerIds.size() > 0) {      // if at least one marker detected
            aruco::drawDetectedMarkers(src_img, markerCorners, markerIds);

            vector<cv::Vec3d> rvecs;
            vector<cv::Vec3d> tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, labelWidth, cameraMatrix, distCoeffs, rvecs, tvecs);

            // 画出轴
            //for (int i = 0; i < markerIds.size(); i++)
            //   cv::aruco::drawAxis(src_img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

            // 旋转矩阵
            Mat rot_mat  = Mat(3,3, CV_32F);
            Rodrigues(rvecs[0], rot_mat);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    if (j < 3) {
                        Rt.at<float>(i, j) = rot_mat.at<float>(i, j);
                    }
                    else {
                        Rt.at<float>(i, j) = tvecs[0][i];
                    }
                }
            }
            Mat res = Mat(3, 1, CV_32F);
            try
            {   
                res = Rt*camera_pos ;
                
            }
            catch (const std::exception& e)
            {
                cout << e.what() << endl;
            }
            cout << res << endl;
            /*String Message = to_string(1) + "," + to_string(2);
            sendMessage(Message);*/
        }
        else {
            cout << "cannot find anymarks" << endl;
        }
        //std::chrono::time_point<std::chrono::high_resolution_clock> p1 = std::chrono::high_resolution_clock::now();
        //cout << "stitch high_resolution_clock time:" << (float)std::chrono::duration_cast<std::chrono::microseconds>(p1 - p0).count() / 1000 << "ms" << endl;
        //=======================

        if (g_bExit) {
            break;
        }
        flag = 3;
        data_var.notify_all();
    }
    return 0;
}


static  unsigned int __stdcall  WorkThread3(void* pUser)
{
    while (1) {
        std::unique_lock<std::mutex> lck(data_mutex);
        data_var.wait(lck, [] {return flag == 3; });
        //std::cout << "thread: " << std::this_thread::get_id() << "   printf: " << "WorkThread3" << std::endl;

        cv::imshow("test", src_img);
        cv::waitKey(1);
        flag = 1;
        if (g_bExit) {
            break;
        }
        data_var.notify_all();
    }
    return 0;
}


int main(int argv, char** argc) {

    //std::cout << "main: " << std::this_thread::get_id() << "   printf: " << "main" << std::endl;
    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST wt_devices;
    memset(&wt_devices, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &wt_devices);
    if (ret != MV_OK) {
        cout << "enum devices faild!" << endl;
        return -1;
    }
    if (wt_devices.nDeviceNum > 0) {
        MV_CC_DEVICE_INFO* wt_camera = wt_devices.pDeviceInfo[0];
        if (printDeviceInfo(wt_camera) == false) {
            return -1;
        }
    }
    else {
        cout << "no device found" << endl;
        return -1;
    }

    ret = MV_CC_CreateHandle(&handle, wt_devices.pDeviceInfo[0]);
    if (ret != MV_OK) {
        return -1;
    }
    ret = MV_CC_OpenDevice(handle);
    if (ret != MV_OK) {
        return -1;
    }

    ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (ret != MV_OK) {
        return -1;
    }

    MVCC_INTVALUE wt_param;
    memset(&wt_param, 0, sizeof(MVCC_INTVALUE));
    ret = MV_CC_GetIntValue(handle, "PayloadSize", &wt_param);
    if (ret != MV_OK) {
        return -1;
    }
    payload_size = wt_param.nCurValue;

    // load config
    ret = MV_CC_FeatureLoad(handle, "FeatureFile.ini");
    if (ret != MV_OK) {
        cout << "loading config file faild" << endl;
        return -1;
    }


    // save config
    /*ret = MV_CC_FeatureSave(handle, "FeatureFile.ini");
    if (ret != MV_OK) {
        return -1;
    }*/

    // start grabbing images
    ret = MV_CC_StartGrabbing(handle);
    if (ret != MV_OK) {
        cout << "grab image failed!" << endl;
        return -1;
    }


    unsigned int nThreadID = 0;
    void* hThreadHandle = (void*)_beginthreadex(NULL, 0, WorkThread, handle, 0, &nThreadID);
    if (NULL == hThreadHandle)
    {
        return -1;
    }

    void* hThreadHandle2 = (void*)_beginthreadex(NULL, 0, WorkThread2, handle, 0, &nThreadID);
    if (NULL == hThreadHandle2)
    {
        return -1;
    }

    void* hThreadHandle3 = (void*)_beginthreadex(NULL, 0, WorkThread3, handle, 0, &nThreadID);
    if (NULL == hThreadHandle3)
    {
        return -1;
    }

    printf("Press a key to stop grabbing.\n");
    WaitForKeyPress();
    g_bExit = true;

    // stop grap image
    ret = MV_CC_StopGrabbing(handle);
    if (ret != MV_OK) {
        return -1;

    }
    // close device
    ret = MV_CC_CloseDevice(handle);
    if (ret != MV_OK) {
        return -1;

    }
    ret = MV_CC_DestroyHandle(handle);
    if (ret != MV_OK) {
        return -1;

    }

    //关闭套接字
    closesocket(s_server);
    //释放DLL资源
    WSACleanup();
    return 0;
}