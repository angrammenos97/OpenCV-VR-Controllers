#include "../headers/Controller.h"

using namespace std;
using namespace cv;




void Controller::initSystem(Controllers& controllers)
{
	initSixense();
	initFreeTrack();
	controllers.serialPorts = new Serial*[2];
	// Right controller
	controllers.sixenseEmulatedData[0].buttons = (0x01 << 0);
	controllers.sixenseEmulatedData[0].controller_index = 0;
	controllers.sixenseEmulatedData[0].enabled = 1;
	controllers.sixenseEmulatedData[0].is_docked = 0;
	controllers.sixenseEmulatedData[0].joystick_x = 0.0f;
	controllers.sixenseEmulatedData[0].joystick_y = 0.0f;
	controllers.sixenseEmulatedData[0].yaw = 0.0f;
	controllers.sixenseEmulatedData[0].pitch = 0.0f;
	controllers.sixenseEmulatedData[0].roll = 0.0f;
	controllers.sixenseEmulatedData[0].trigger = 0.0f;
	controllers.sixenseEmulatedData[0].which_hand = 'L';
	controllers.sixenseEmulatedData[0].x = -65.0f;
	controllers.sixenseEmulatedData[0].y = -45.0f;
	controllers.sixenseEmulatedData[0].z = -200.0f;
	// Left controller
	controllers.sixenseEmulatedData[1].buttons = (0x01 << 0);
	controllers.sixenseEmulatedData[1].controller_index = 1;
	controllers.sixenseEmulatedData[1].enabled = 1;
	controllers.sixenseEmulatedData[1].is_docked = 0;
	controllers.sixenseEmulatedData[1].joystick_x = 0.0f;
	controllers.sixenseEmulatedData[1].joystick_y = 0.0f;
	controllers.sixenseEmulatedData[1].yaw = 0.0f;
	controllers.sixenseEmulatedData[1].pitch = 0.0f;
	controllers.sixenseEmulatedData[1].roll = 0.0f;
	controllers.sixenseEmulatedData[1].trigger = 0.0f;
	controllers.sixenseEmulatedData[1].which_hand = 'R';
	controllers.sixenseEmulatedData[1].x = 65.0f;
	controllers.sixenseEmulatedData[1].y = -45.0f;
	controllers.sixenseEmulatedData[1].z = -200.0f;
	//Head tracker
	controllers.freeTrackData.CamHeight = 1080;
	controllers.freeTrackData.CamWidth = 1920;
	controllers.freeTrackData.DataID = 0;
	controllers.freeTrackData.Yaw = 0.0f;
	controllers.freeTrackData.Pitch = 0.0f;
	controllers.freeTrackData.Roll = 0.0f;
	controllers.freeTrackData.RawYaw = 0.0f;
	controllers.freeTrackData.RawPitch = 0.0f;
	controllers.freeTrackData.RawRoll = 0.0f;
	controllers.freeTrackData.RawX = 0.0f;
	controllers.freeTrackData.RawY = 0.0f;	
	controllers.freeTrackData.RawZ = 0.0f;	
	controllers.freeTrackData.X = 0.0f;
	controllers.freeTrackData.X1 = 0.0f;
	controllers.freeTrackData.X2 = 0.0f;
	controllers.freeTrackData.X3 = 0.0f;
	controllers.freeTrackData.X4 = 0.0f;
	controllers.freeTrackData.Y = 0.0f;
	controllers.freeTrackData.Y1 = 0.0f;
	controllers.freeTrackData.Y2 = 0.0f;
	controllers.freeTrackData.X3 = 0.0f;
	controllers.freeTrackData.Y4 = 0.0f;	
	controllers.freeTrackData.Z = 0.0f;
	sixenseSetData(controllers.sixenseEmulatedData);
	controllers.sixenseEmulatedData[0].buttons = 0x00;
	controllers.sixenseEmulatedData[1].buttons = 0x00;
	sixenseSetData(controllers.sixenseEmulatedData);
	freetrackSetData(controllers.freeTrackData);
}

void Controller::setColor(int color[][2], Controllers& controller, int which , string port)
{
	if (Controller::debug) {
		if (which == 0)
			cout << "[DEBUG:Setting Right controller's color]" << endl;
		else if (which == 1)
			cout << "[DEBUG:Setting Left controller's color]" << endl;
		else if (which == 2)
			cout << "[DEBUG:Setting Head tracker's color]" << endl;
	}
	controller.color[which][0][0] = color[0][0];
	controller.color[which][0][1] = color[0][1];
	controller.color[which][1][0] = color[1][0];
	controller.color[which][1][1] = color[1][1];
	controller.color[which][2][0] = color[2][0];
	controller.color[which][2][1] = color[2][1];

	controller.addedControllers[which] = true;
	controller.ports[which] = port;
}

void Controller::setPosition(Controllers& controller, int which, Mat pnts3D, bool isItHomo)
{
	float sens = 10;
	if (isItHomo)
	{
		if (which == 0) {
			controller.sixenseEmulatedData[0].x = sens* ( pnts3D.at<float>(0, 0) / pnts3D.at<float>(3, 0));
			controller.sixenseEmulatedData[0].y = sens* ( pnts3D.at<float>(2, 0) / pnts3D.at<float>(3, 0));
			controller.sixenseEmulatedData[0].z = -sens* ( pnts3D.at<float>(1, 0) / pnts3D.at<float>(3, 0));
			// Just to test
			{controller.sixenseEmulatedData[1].x = sens* ((pnts3D.at<float>(0, 0) / pnts3D.at<float>(3, 0))) + 130;
			controller.sixenseEmulatedData[1].y = sens* (pnts3D.at<float>(2, 0) / pnts3D.at<float>(3, 0));
			controller.sixenseEmulatedData[1].z = -sens* (pnts3D.at<float>(1, 0) / pnts3D.at<float>(3, 0));
			controller.freeTrackData.X = sens* (pnts3D.at<float>(0, 0) / pnts3D.at<float>(3, 0));
			controller.freeTrackData.Y = sens* (pnts3D.at<float>(1, 0) / pnts3D.at<float>(3, 0));
			controller.freeTrackData.Z = sens* (pnts3D.at<float>(2, 0) / pnts3D.at<float>(3, 0)); }
		}
		else if (which == 1) {
			controller.sixenseEmulatedData[1].x = sens* (pnts3D.at<float>(0, 0) / pnts3D.at<float>(3, 0));
			controller.sixenseEmulatedData[1].y = sens* (pnts3D.at<float>(2, 0) / pnts3D.at<float>(3, 0));
			controller.sixenseEmulatedData[1].z = -sens* (pnts3D.at<float>(1, 0) / pnts3D.at<float>(3, 0));
		}
		else if (which == 2) {
			controller.freeTrackData.X = pnts3D.at<float>(0, 0) / pnts3D.at<float>(3, 0);
			controller.freeTrackData.Y = pnts3D.at<float>(1, 0) / pnts3D.at<float>(3, 0);
			controller.freeTrackData.Z = pnts3D.at<float>(2, 0) / pnts3D.at<float>(3, 0);
		}
		else {
			cout << "[ERROR: Setting position on " << which << " !]" << endl;
		}
	}
	else
	{
		if (which == 0) {
			controller.sixenseEmulatedData[0].x = sens* (pnts3D.at<double>(0, 0));
			controller.sixenseEmulatedData[0].y = sens* (pnts3D.at<double>(2, 0));
			controller.sixenseEmulatedData[0].z = -sens* (pnts3D.at<double>(1, 0));
			// Just to test
			{controller.sixenseEmulatedData[1].x = sens* (pnts3D.at<double>(0, 0)) + 130;
			controller.sixenseEmulatedData[1].y = sens* (pnts3D.at<double>(2, 0));
			controller.sixenseEmulatedData[1].z = -sens* (pnts3D.at<double>(1, 0));
			controller.freeTrackData.X = sens* (pnts3D.at<double>(0, 0));
			controller.freeTrackData.Y = sens* (pnts3D.at<double>(1, 0));
			controller.freeTrackData.Z = sens* (pnts3D.at<double>(2, 0)); }
		}
		else if (which == 1) {
			controller.sixenseEmulatedData[1].x = sens* (pnts3D.at<double>(0, 0));
			controller.sixenseEmulatedData[1].y = sens* (pnts3D.at<double>(2, 0));
			controller.sixenseEmulatedData[1].z = -sens* (pnts3D.at<double>(1, 0));
		}
		else if (which == 2) {
			controller.freeTrackData.X = pnts3D.at<double>(0, 0);
			controller.freeTrackData.Y = pnts3D.at<double>(1, 0);
			controller.freeTrackData.Z = pnts3D.at<double>(2, 0);
		}
		else {
			cout << "[ERROR: Setting position on " << which << " !]" << endl;
		}
	}

	if (Controller::debug) {
		if (which == 0)
			cout << "[DEBUG:Right controller has position ( " << controller.sixenseEmulatedData[0].x << " , " << controller.sixenseEmulatedData[0].y << " , " << controller.sixenseEmulatedData[0].z << "]" << endl;
		else if (which == 1)
			cout << "[DEBUG:Left controller has position ( " << controller.sixenseEmulatedData[1].x << " , " << controller.sixenseEmulatedData[1].y << " , " << controller.sixenseEmulatedData[1].z << "]" << endl;
		else if (which == 2)
			cout << "[DEBUG:Head tracker has position ( " << controller.freeTrackData.X << " , " << controller.freeTrackData.Y << " , " << controller.freeTrackData.Z << "]" << endl;
	}
}

void Controller::getSerialData(Controllers& controller, int which)
{
	ArduinoData arduino_data = { 0 };
	if (!controller.serialPorts[which]->IsConnected()) {
		if (which == 0)
			cout << "ERROR:Serial of the Right controller is not open." << endl;
		else if (which == 1)
			cout << "ERROR:Serial of the Left controller is not open." << endl;
		return;
	}
	char buff[sizeof(ArduinoData)] = { 0 };
	controller.serialPorts[which]->ReadData(buff, sizeof(ArduinoData));
	memcpy(&arduino_data.buttons, buff, sizeof(ArduinoData));
	if (Controller::debug)
		cout << "Serial " << controller.serialPorts[which]->portName << " : " << arduino_data.buttons << "\t( " << arduino_data.joyx << " | " << arduino_data.joyy << " )" << endl;	

	controller.sixenseEmulatedData[which].buttons = arduino_data.buttons;
	controller.sixenseEmulatedData[which].joystick_x = arduino_data.joyx;
	controller.sixenseEmulatedData[which].joystick_y = arduino_data.joyy;

	if ((arduino_data.buttons & (0x01 << 5)) == (0x01 << 5)) {
		controller.sixenseEmulatedData[0].x = -65.0f;
		controller.sixenseEmulatedData[0].y = -45.0f;
		controller.sixenseEmulatedData[0].z = -200.0f;
		controller.sixenseEmulatedData[0].x = 65.0f;
		controller.sixenseEmulatedData[0].y = -45.0f;
		controller.sixenseEmulatedData[0].z = -200.0f;
		controller.sixenseEmulatedData[0].buttons = (0x01 << 0);
		controller.sixenseEmulatedData[1].buttons = (0x01 << 0);
		cout << "Controllers calibrated!" << endl;
	}

	if (Controller::debug)
		cout << "[DEBUG:Serial Done]" << endl;
}

/*Controller::Controller(string name)
{
	this->name = name;
	this->color[0][0] = 0;
	this->color[0][1] = 255;
	this->color[1][0] = 0;
	this->color[1][1] = 255;
	this->color[2][0] = 0;
	this->color[2][1] = 255;

	this->position.push_back(0.0f);
	this->position.push_back(0.0f);
	this->position.push_back(0.0f);
}


Controller::~Controller()
{
}

string Controller::getName()
{
	return this->name;
}

void Controller::setColor(int color[3][2])
{
	this->color[0][0] = color[0][0];
	this->color[0][1] = color[0][1];
	this->color[1][0] = color[1][0];
	this->color[1][1] = color[1][1];
	this->color[2][0] = color[2][0];
	this->color[2][1] = color[2][1];
	cout << "The color from " << this->name << " just saved!" << endl;
}


void Controller::setPosition(Mat pnts3D, bool isItHomo)
{
	if (this->debug)
		cout << "[DEBUG:Controller: " << this->name << " sets its posiotion]" << endl;
	if (isItHomo)
	{
		this->position[0] = pnts3D.at<float>(0, 0) / pnts3D.at<float>(3, 0);
		this->position[1] = pnts3D.at<float>(1, 0) / pnts3D.at<float>(3, 0);
		this->position[2] = pnts3D.at<float>(2, 0) / pnts3D.at<float>(3, 0);
	}
	else
	{
		this->position[0] = pnts3D.at<double>(0, 0);
		this->position[1] = pnts3D.at<double>(1, 0);
		this->position[2] = pnts3D.at<double>(2, 0);
	}
	if (this->debug)
		cout << this->name << " 's positions is: (" << this->position[0] << " , " << this->position[1] << " , " << this->position[2] << ")." << endl;
}

vector<float> Controller::getPosition()
{
	return this->position;
}*/