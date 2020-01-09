#pragma once
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Controller.h"

class Camera
{
	cv::VideoCapture cap;	
	cv::Mat rvec = cv::Mat(1, 3, cv::DataType<float>::type);
	cv::Mat tvec = cv::Mat(1, 3, cv::DataType<float>::type);
	cv::Mat rotationMatrix = cv::Mat(3, 3, cv::DataType<float>::type);
	cv::Mat cameraPosition = cv::Mat(3, 3, cv::DataType<float>::type);
	cv::Point3f cameraRotation;
	std::vector<cv::Point2f> imagePoints;
	cv::Mat imgOriginal;
	cv::Mat imgThresholded;
	float radius;

public:
	int source;
	static bool debug;		
	static bool displayFrame;
	static bool displayMask;
	static bool foundCameraPosition;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat projectionMatrix = cv::Mat::zeros(3, 4, cv::DataType<double>::type);
	cv::Mat position;
	cv::Point2f center;

	Camera(int source , bool displayFrame = true);
	~Camera();
	bool takeFrame();
	cv::Size getFrameSize();
	void takeMaskFrame(int color[3][2]);
	void showFrame();
	void showMask();
	bool trackObject(int color[][2] , int which, bool fastWay);
	void captureObject(int color[][2] , bool capture);
	void findCameraPosition(std::vector<cv::Point3f> objectPoints);
	void calculateProjectionMatrix();
};