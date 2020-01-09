#pragma once
#include <opencv2/core/core.hpp>
#include "AllCameras.h"
#include "Controller.h"

using namespace cv;

class ParallelProcess : public ParallelLoopBody
{
	AllCameras &myCamera;
	vector<Controller>& myControllers;

public:
	ParallelProcess(AllCameras &cameras, vector<Controller>& controllers)
		: myCamera(cameras), myControllers(controllers) {}

	virtual void operator()(const cv::Range& range) const
	{
		for (int j = range.start; j < range.end; j++)
		{
			myCamera.camera[j].takeFrame();
			//myCamera.camera[j].trackObject(myControllers);
		}
	}
};