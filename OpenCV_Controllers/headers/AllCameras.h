#pragma once
#include <vector>
#include "Camera.h"
#include "Controller.h"

class AllCameras
{	

public:
	static bool debug;
	std::vector<Camera> camera;

	AllCameras();	
	~AllCameras();
	bool addNewCamera(Camera camera);
	bool setColorValues(Controllers& controllers , int which , std::string port);
	void trackObject(Controllers& controllers);
	void showCamerasFrames(bool show);
	void setDisplayFrames(bool show);
	void findCamerasPosition(int color[][2]);
};

