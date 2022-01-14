#include <iostream>
#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include <conio.h>
#include "opencv2/aruco.hpp"

#include "json/json.h"

// 多线程
#include <mutex>
#include <condition_variable>

// socket
#include<winsock.h>
#pragma comment(lib,"ws2_32.lib")

using namespace cv;


std::mutex data_mutex;
std::condition_variable data_var;
int flag = 1;


cv::Mat src_img;
// RT矩阵
cv::Mat Rt = cv::Mat::zeros(4, 4, CV_64F);
// 相机世界坐标
Mat camera_world;

bool print_xyz = false;

// 标签大小
float labelWidth = 0.034;

unsigned int payload_size = 0;
bool g_bExit = false;

int ret = MV_OK;

// socket 
int send_len = 0;
int recv_len = 0;
int len = 0;
//定义发送缓冲区和接受缓冲区
char send_buf[1024];
char recv_buf[1024];
//定义服务端套接字，接受请求套接字
SOCKET s_server;
SOCKET s_accept;
//服务端地址客户端地址
SOCKADDR_IN server_addr;
SOCKADDR_IN accept_addr;

void initialization() {
    //初始化套接字库
    WORD w_req = MAKEWORD(2, 2);//版本号
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        std::cout << "初始化套接字库失败！" << std::endl;
    }
    else {
        std::cout << "初始化套接字库成功！" << std::endl;
    }
    //检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        std::cout << "套接字库版本号不符！" << std::endl;
        WSACleanup();
    }
    else {
        std::cout << "套接字库版本正确！" << std::endl;
    }
    //填充服务端地址信息
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
std::vector<cv::Point3f> getCornersInCameraWorld(double side, cv::Vec3d rvec, cv::Vec3d tvec) {

    double half_side = side / 2;


    // compute rot_mat
    cv::Mat rot_mat;
    Rodrigues(rvec, rot_mat);

    // transpose of rot_mat for easy columns extraction
    cv::Mat rot_mat_t = rot_mat.t();

    // the two E-O and F-O vectors
    double* tmp = rot_mat_t.ptr<double>(0);
    cv::Point3f camWorldE(tmp[0] * half_side,
        tmp[1] * half_side,
        tmp[2] * half_side);

    tmp = rot_mat_t.ptr<double>(1);
    cv::Point3f camWorldF(tmp[0] * half_side,
        tmp[1] * half_side,
        tmp[2] * half_side);

    // convert tvec to point
    cv::Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

    // return vector:
    std::vector<cv::Point3f> ret(4, tvec_3f);

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
    //std::std::cout <<  std::this_thread::get_id() << "   printf: " << "WorkThread1" << std::std::endl;
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


static  unsigned int __stdcall  WorkThread2(void* pUser)
{
    while (1) {
        std::unique_lock<std::mutex> lck(data_mutex);
        data_var.wait(lck, [] {return flag == 2; });

        //std::std::cout << "thread: " << std::this_thread::get_id() << "   printf: " << "WorkThread2" << std::std::endl;
        //std::chrono::time_point<std::chrono::high_resolution_clock> p0 = std::chrono::high_resolution_clock::now();
        //========处理检测码============
         // load intrinsics,需要标定相机才能获取
        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F), distCoeffs = cv::Mat(1, 5, CV_32F);
        
        std::vector<std::vector<float>> intrinsics = {
            {3535.78,0,860.79} ,
            {0,3551.96,480.73},
            {0,0,1}
        };
        std::vector<std::vector<float>> distCoeffsMat = {
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

        cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_50));
        // board: aruco map. create(x_num, y_num, size(m), gap, diction, (index=1))
        cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(7, 10, labelWidth, 0.004, dict);   // real distance in meters.
        std::vector<int> markerIds;         // detected ids.
        std::vector<std::vector<cv::Point2f>> markerCorners;
        try {
            cv::aruco::detectMarkers(src_img, board->dictionary, markerCorners, markerIds);
        }
        catch (Exception e) {
            std::cout << e.what() << std::endl;
        }
        

        if (markerIds.size() > 0) {      // if at least one marker detected
            cv::aruco::drawDetectedMarkers(src_img, markerCorners, markerIds);

            std::vector<cv::Vec3d> rvecs;
            std::vector<cv::Vec3d> tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, labelWidth, cameraMatrix, distCoeffs, rvecs, tvecs);

            // 画出轴
            //for (int i = 0; i < markerIds.size(); i++)
            //   cv::aruco::drawAxis(src_img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            //y *=-1
            tvecs[0][1] *= -1;
            // 旋转矩阵
            Mat rot_mat;
            Rodrigues(rvecs[0], rot_mat);
            //for (int i = 0; i < 3; i++)
            //{
            //    for (int j = 0; j < 3; j++)
            //    {
            //        std::cout << rot_mat.at<double>(i, j) << " ";
            //    }
            //    std::cout << std::endl;
            //}
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Rt.at<double>(i, j) = rot_mat.at<double>(i, j);
                }
            }
            for (int j = 0; j < 3; j++) {
                // 赋值平移
                Rt.at<double>(j, 3) = tvecs[0][j];
            }
            Rt.at<double>(3, 3) = 1;

 /*           for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    std::cout << Rt.at<double>(i, j) << " ";
                }
                std::cout << std::endl;
            }*/
        }
        else {
            //std::cout << "cannot find anymarks" << std::endl;
        }
        //std::chrono::time_point<std::chrono::high_resolution_clock> p1 = std::chrono::high_resolution_clock::now();
        //std::cout << "stitch high_resolution_clock time:" << (float)std::chrono::duration_cast<std::chrono::microseconds>(p1 - p0).count() / 1000 << "ms" << std::endl;
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
        //std::std::cout << "thread: " << std::this_thread::get_id() << "   printf: " << "WorkThread3" << std::std::endl;

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


static  unsigned int __stdcall  WorkThread4(void* pUser)
{   
    //填充服务端信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("172.22.25.111");
    server_addr.sin_port = htons(2022);
    //创建套接字
    s_server = socket(AF_INET, SOCK_STREAM, 0);
    int flag = bind(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR));
    if (flag == SOCKET_ERROR) {
        std::cout << "套接字绑定失败！" << std::endl;
        WSACleanup();
    }
    else {
        std::cout << "套接字绑定成功！" << std::endl;
    }
    //设置套接字为监听状态
    if (listen(s_server, SOMAXCONN) < 0) {
        std::cout << "设置监听状态失败！" << std::endl;
        WSACleanup();
    }
    else {
        std::cout << "设置监听状态成功！" << std::endl;
    }
    std::cout << "服务端正在监听连接，请稍候...." << std::endl;
    //接受连接请求
    len = sizeof(SOCKADDR);
    s_accept = accept(s_server, (SOCKADDR*)&accept_addr, &len);
    if (s_accept == SOCKET_ERROR) {
        std::cout << "连接失败！" << std::endl;
        WSACleanup();
        return 0;
    }
    std::cout << "连接建立，准备接受数据" << std::endl;

    while (1) {
       
        recv_len = recv(s_accept, recv_buf, 1024, 0);
        if (recv_len < 0) {
            std::cout << "接受失败！" << std::endl;
            continue;
        }
        else {
            std::cout << recv_buf << std::endl;
            std::string str(recv_buf);
            std::vector<double> position;
            std::stringstream ss(str);
            double temp;
            while (ss >> temp)
                position.push_back(temp);
            Mat marker_world = Mat::ones(4, 1, CV_64F);
            for (int j = 0; j < 3; j++)
            {
                marker_world.at<double>(j,0) = position[j];
            }
            Mat invert_Rt;
            invert(Rt, invert_Rt);
            camera_world = invert_Rt*marker_world;
            print_xyz = true;
            /*double x = camera_world.at<double>(0, 0);
            double y = camera_world.at<double>(1, 0);
            double z = camera_world.at<double>(2, 0);
            std::cout << x << std::endl;
            std::cout << y << std::endl;
            std::cout << z << std::endl;*/

        }
        send_len = send(s_accept, "123", 1024, 0);
        if (send_len < 0) {
            std::cout << "发送失败！" << std::endl;
            continue;
        }
    }
    return 0;
}

static  unsigned int __stdcall  WorkThread5(void* pUser) {
    while (1) {
        if (print_xyz) {
            Mat new_marker_world = Rt * camera_world;
            double x = new_marker_world.at<double>(0, 0);
            double y = new_marker_world.at<double>(1, 0);
            double z = new_marker_world.at<double>(2, 0);
            std::string position = std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
            send_len = send(s_accept, position.data(), 1024, 0);
            if (send_len < 0) {
                std::cout << "发送失败！" << std::endl;
                continue;
            }
            
        }
    }
    return 0;
}




int main() {
   
    initialization();
    //std::std::cout << "main: " << std::this_thread::get_id() << "   printf: " << "main" << std::std::endl;
    void* handle = NULL;
    MV_CC_DEVICE_INFO_LIST wt_devices;
    memset(&wt_devices, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &wt_devices);
    if (ret != MV_OK) {
        std::cout << "enum devices faild!" << std::endl;
        return -1;
    }
    if (wt_devices.nDeviceNum > 0) {
        MV_CC_DEVICE_INFO* wt_camera = wt_devices.pDeviceInfo[0];
        if (printDeviceInfo(wt_camera) == false) {
            return -1;
        }
    }
    else {
        std::cout << "no device found" << std::endl;
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
        std::cout << "loading config file faild" << std::endl;
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
        std::cout << "grab image failed!" << std::endl;
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

    void* hThreadHandle_socket = (void*)_beginthreadex(NULL, 0, WorkThread4, handle, 0, &nThreadID);
    if (NULL == hThreadHandle_socket)
    {
        return -1;
    }
    unsigned int nThreadID5 = 10;
    void* hThreadHandle_print = (void*)_beginthreadex(NULL, 0, WorkThread5, handle, 0, &nThreadID5);
    if (NULL == hThreadHandle_print)
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
    closesocket(s_accept);
    //释放DLL资源
    WSACleanup();
    return 0;
}