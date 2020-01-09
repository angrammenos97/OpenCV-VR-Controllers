#include "headers\Functions.h"
#include <ctime>

using namespace std;
using namespace cv;
using namespace Functions;


bool debug = false;
bool Camera::debug = false;
bool AllCameras::debug = false;
//bool Controller::debug = false;
bool Camera::displayFrame = true;
bool Camera::displayMask = false;
bool Camera::foundCameraPosition = false;

bool exitProgram = false;
bool startProcess = false;
AllCameras myCameras;
Controllers AllControllers;
Mat mainMenuImage;
Mat mainMenuEffect;


/*void addCamera()
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

void addController()
{
	if (myCameras.camera.size() == 0)
	{
		cout << "You must add a camera first!" << endl << endl;
		return;
	}
	string tempControllerName;	
	std::cout << "Give the controller's name(-1 to cancel): ";
	cin >> tempControllerName;
	if (tempControllerName == "-1")
	{
		std::cout << "Add controller canceled.." << endl << endl;
		return;
	}
	Controller tempController = Controller(tempControllerName);
	if (myCameras.setColorValues(tempController))
		controllers.push_back(tempController);

	std::cout << "Done!" << endl << endl;
}

void displayFrames()
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

void start_stopProcess()
{
	if (startProcess)
	{
		std::cout << "Stop Process." << endl << endl;
		startProcess = false;
	}
	else
	{
		if (controllers.size() == 0)
		{
			std::cout << "You must add a controller first!" << endl << endl;
			return;
		}
		if (!Camera::foundCameraPosition)
		{
			cout << "Starting calibrating camera positions.." << endl;
			cout << "Take the " << controllers[0].getName() << " controller and place it on paper's spots." << endl;
			cout << "Press Enter to capture every single spot" << endl;
			myCameras.findCamerasPosition(controllers[0]);
			Camera::foundCameraPosition = true;
		}
		std::cout << "Start Process." << endl << endl;
		startProcess = true;
	}	
}*/

static void onMouse(int event, int x, int y, int, void*)
{
	if (x > 35 && x < 190)
	{
		if (y > 70 && y < 105)
		{
			//mainMenuEffect = Mat::zeros(mainMenuEffect.size(), CV_8UC3);
			rectangle(mainMenuEffect , Rect(35 , 70 , 186-35 , 105-70) , Scalar(255,0,0) , 4);
			if (event == EVENT_LBUTTONDOWN)
				addCamera(myCameras);
		}
		else if (y > 130 && y < 165)
		{
			rectangle(mainMenuEffect, Rect(35, 130, 186 - 35, 165 - 130), Scalar(255, 0, 0), 4);
			if (event == EVENT_LBUTTONDOWN)
				addController(myCameras , AllControllers);
		}
		else if (y > 190 && y < 225)
		{
			rectangle(mainMenuEffect, Rect(35, 190, 186 - 35, 225 - 190), Scalar(255, 0, 0), 4);
			if (event == EVENT_LBUTTONDOWN)
				displayFrames();
		}
		else if (y > 245 && y < 275)
		{
			rectangle(mainMenuEffect, Rect(35, 245, 186 - 35, 275 - 245), Scalar(255, 0, 0), 4);
			if (event == EVENT_LBUTTONDOWN)
				start_stopProcess(myCameras , AllControllers, startProcess);
		}
		else if (y > 310 && y < 345)
		{
			rectangle(mainMenuEffect, Rect(35, 310, 186 - 35, 345 - 310), Scalar(255, 0, 0), 4);
			if (event == EVENT_LBUTTONDOWN)
				exitProgram = true;
		}
		else
			mainMenuEffect = Mat::zeros(mainMenuEffect.size(), CV_8UC3);
	}
	else
		mainMenuEffect = Mat::zeros(mainMenuEffect.size(), CV_8UC3);

	imshow("Main Menu", mainMenuImage + mainMenuEffect);
}


int main(int argc, char *argv)
{
	{
		Camera::debug = debug;
		AllCameras::debug = debug;
		Controller::debug = debug;
	}

	std::cout << "Hello. Choose an option from 'Main Menu' window." << endl << endl;
	mainMenuImage = imread("./data/menu.jpg", CV_LOAD_IMAGE_COLOR);
	mainMenuEffect = Mat::zeros(mainMenuImage.size(), CV_8UC3);
	namedWindow("Main Menu", CV_WINDOW_AUTOSIZE);
	imshow("Main Menu", mainMenuImage);
	setMouseCallback("Main Menu", onMouse , 0);

	cout << "Initializing shared memory files ..." << std::endl;
	Controller::initSystem(AllControllers);	
	//Serial *SP = new Serial("\\\\.\\COM5");
	//char incomingData[sizeof(ArduinoData)];
	//ArduinoData arduino_data = { 0 };
	cout << "Initializied compleded!" << endl << endl;

	clock_t start;
	double dt;
	while (!exitProgram)
	{
		start = clock();
		if (waitKey(1) == -1)
		{
			imshow("Main Menu", mainMenuImage + mainMenuEffect);
			setMouseCallback("Main Menu", onMouse, 0);
		}
		if (!startProcess)
			myCameras.showCamerasFrames(Camera::displayFrame);
		else
		{
			//if (SP->clearBuffer())
			//	if (debug)
			//		cout << "[DEBUG:Serial cleared]" << endl;

			myCameras.trackObject(AllControllers);
			//parallel_for_(Range(0, myCameras.camera.size()), ParallelProcess(myCameras, controllers));

			//arduino_data = { 0 };

			//SP->ReadData(incomingData, sizeof(ArduinoData));
			//memcpy(&arduino_data.buttons, incomingData, sizeof(ArduinoData));
			//cout << "Serial: " << arduino_data.buttons << "\t(" << arduino_data.joyx << "," << arduino_data.joyy << ")" << endl;

			freetrackSetData(AllControllers.freeTrackData);
			sixenseSetData(AllControllers.sixenseEmulatedData);

		}
		dt = (clock() - start) / (double)CLOCKS_PER_SEC;
		cout << "Freq: " << 1 / dt << "." << endl;
	}

	std::cout << "Exiting.." << endl;
	cv::destroyAllWindows();
	// stop run

	return 0;
}