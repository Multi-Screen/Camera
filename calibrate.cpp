#include <iostream>
#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "CameraCalibrator.h"

//int main()
//{
//
//	cv::namedWindow("Image");
//	cv::Mat image;
//	std::vector<std::string> filelist;
//
//	// generate list of chessboard image filename
//	for (int i=1; i<=20; i++) {
//
//		std::stringstream str;
//		str << "./chessboards/chessboard" << std::setw(2) << std::setfill('0') << i << ".bmp";
//		std::cout << str.str() << std::endl;
//
//		filelist.push_back(str.str());
//		image= cv::imread(str.str(),0);
//		cv::imshow("Image",image);
//	
//		cv::waitKey(100);
//	}
//
//	// Create calibrator object
//    CameraCalibrator cameraCalibrator;
//	// add the corners from the chessboard
//	cv::Size boardSize(9,6);
//	cameraCalibrator.addChessboardPoints(
//		filelist,	// filenames of chessboard image
//		boardSize);	// size of chessboard
//		// calibrate the camera
//    //	cameraCalibrator.setCalibrationFlag(true,true);
//	cv::Size s = image.size();
//	cameraCalibrator.calibrate(s);
//
//    // Image Undistortion
//    image = cv::imread(filelist[6]);
//	cv::Mat uImage= cameraCalibrator.remap(image);
//
//	// display camera matrix
//	cv::Mat cameraMatrix= cameraCalibrator.getCameraMatrix();
//	std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
//	std::cout << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << std::endl;
//	std::cout << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << std::endl;
//	std::cout << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << std::endl;
//
//    
//	cv::Mat distCoeffs =  cameraCalibrator.getDistCoeffs();
//	std::cout << " distCoeffs: " << distCoeffs.rows << "x" << distCoeffs.cols << std::endl;
//	std::cout << distCoeffs.at<double>(0, 0) << " " << distCoeffs.at<double>(0, 1) << " " << distCoeffs.at<double>(0, 2) << std::endl;
//	std::cout << distCoeffs.at<double>(1, 0) << " " << distCoeffs.at<double>(1, 1) << " " << distCoeffs.at<double>(1, 2) << std::endl;
//	std::cout << distCoeffs.at<double>(2, 0) << " " << distCoeffs.at<double>(2, 1) << " " << distCoeffs.at<double>(2, 2) << std::endl;
//	
//	imshow("Original Image", image);
//    imshow("Undistorted Image", uImage);
//
//	cv::waitKey();
//	return 0;
//}