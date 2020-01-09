#include "../headers/Functions.h"
#include <string>

using namespace std;

void Functions::addCamera(AllCameras &myCameras)
{
	int tempCameraID;
	do
	{
		std::cout << "Select camera(e.g. 0 for computer's default): ";
		cin >> tempCameraID;
		if (cin.fail())
		{
			cin.clear();
			cin.ignore(256, '\n');
			tempCameraID = -1;
		}
		if (tempCameraID < 0)
		{
			std::cout << "Add camera canceled.." << endl << endl;
			return;
		}

	} while (!myCameras.addNewCamera(Camera(tempCameraID)));
	std::cout << "Done!" << endl << endl;
}

void Functions::addController(AllCameras &myCameras , Controllers& controllers)
{
	if (myCameras.camera.size() == 0){
		std::cout << "You must add a camera first!" << endl << endl;
		return;
	}
	int which = 0 ;
	string tempControllerPort;
	if (!controllers.addedControllers[0]) {
		std::cout << "Give the Right controller's COM Port: ";
		which = 0;
	}
	else if (!controllers.addedControllers[1]){
		std::cout << "Give the Left controller's COM Port: ";
		which = 1;
	}
	else if (!controllers.addedControllers[2]){
		std::cout << "Give the Head's COM Port: ";
		which = 2;
	}
	cin >> tempControllerPort;
	if ( tempControllerPort.find("COM") == string::npos){
		std::cout << "Invalid input" << endl;
		std::cout << "Add controller canceled.." << endl << endl;
		return;
	}
	if (which == 0) {
		tempControllerPort = "\\\\.\\" + tempControllerPort;
		const char *temp = tempControllerPort.c_str();
		controllers.serialPorts[which] = new Serial(temp);
		//Serial rightSerial = Serial(temp);
		if (!controllers.serialPorts[which]->IsConnected()) {
			std::cout << "Can't open serial " << tempControllerPort << endl;
			std::cout << "Add controller canceled.." << endl << endl;
			//controllers.serialPorts.pop_back();
			return;
		}
		//controllers.serialPorts.push_back(rightSerial);
	}
	else if (which == 1) {
		tempControllerPort = "\\\\.\\" + tempControllerPort;
		const char *temp = tempControllerPort.c_str();
		controllers.serialPorts[which] = new Serial(temp);
		//controllers.serialPorts.push_back(Serial(temp));
		//Serial leftSerial = Serial(temp);
		if (!controllers.serialPorts[which]->IsConnected()) {
			std::cout << "Can't open serial " << tempControllerPort << endl;
			std::cout << "Add controller canceled.." << endl << endl;
			//controllers.serialPorts.pop_back();
			return;
		}
		//controllers.serialPorts.push_back(leftSerial);
	}

	myCameras.setColorValues(controllers, which , tempControllerPort);

	std::cout << "Done!" << endl << endl;
}

void Functions::displayFrames()
{
	if (Camera::displayFrame)
	{
		std::cout << "Stop displaying frames" << endl << endl;
		Camera::displayFrame = false;
	}
	else
	{
		std::cout << "Start displaying frames" << endl << endl;
		Camera::displayFrame = true;
	}
}

void Functions::start_stopProcess(AllCameras &myCameras , Controllers controllers , bool& startProcess)
{
	if (startProcess)
	{
		std::cout << "Stop Process." << endl << endl;
		startProcess = false;
	}
	else
	{
		if (controllers.addedControllers == 0)
		{
			std::cout << "You must add a controller first!" << endl << endl;
			return;
		}
		if (!Camera::foundCameraPosition)
		{
			std::cout << "Starting calibrating camera positions.." << endl;
			std::cout << "Take the Right controller and place it on paper's spots." << endl;
			std::cout << "Press Enter to capture every single spot" << endl;
			myCameras.findCamerasPosition(controllers.color[0]);
			Camera::foundCameraPosition = true;
		}
		cout << "Start Process." << endl << endl;
		startProcess = true;
	}
}

