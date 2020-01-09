#pragma once
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
//#include <iostream>
#include <array>
#include "AllCameras.h"
#include "sixense.h"
#include "FreeTrackClient.h"
#include "SerialClass.h"

namespace Functions {

	void addCamera(AllCameras &myCameras);
	void addController(AllCameras &myCameras, Controllers& controllers);
	void displayFrames();
	void start_stopProcess(AllCameras &myCameras, Controllers controllers, bool& startProcess);

}


#endif // !_FUNCTIONS_H_

