#pragma once
#include <iostream>
#include <string>
#include <opencv2\core\core.hpp>
#include "sixense.h"
#include "FreeTrackClient.h"
#include "SerialClass.h"

typedef struct Controllers {	
	bool addedControllers[3];		// {Right Left Head} (true -> added , false -> not added yet)
	int color[3][3][2];
	Serial **serialPorts;
	std::string ports[2];
	FreeTrackData freeTrackData;
	std::array<SixenseEmulatedData , 2> sixenseEmulatedData;
}Controllers;

typedef struct ArduinoData {
	unsigned int buttons;
	float joyx;
	float joyy;
}ArduinoData;

namespace Controller {
	static bool debug;
	void initSystem(Controllers& controllers);
	void setColor(int color[][2], Controllers& controller, int which , std::string port);
	void setPosition(Controllers& controller, int which , cv::Mat pnts3D, bool isItHomo = true);
	void getSerialData(Controllers& controller , int which);
}

/*class Controller
{
	string name;
public:
	static bool debug;
	int color[3][2]; // Hue(low,high) , Saturation(low,high) , Value(low,high)
	vector<float> position;

	Controller(string name);
	~Controller();
	string getName();
	void setColor(int color[3][2]);
	void setPosition(Mat pnts3D , bool isItHomo = true);
	vector<float> getPosition();
};
*/

