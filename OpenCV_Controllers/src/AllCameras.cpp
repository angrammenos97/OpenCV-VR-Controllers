#include "../headers/AllCameras.h"

using namespace std;
using namespace cv;


AllCameras::AllCameras()
{
}

AllCameras::~AllCameras()
{
}

bool AllCameras::addNewCamera(Camera camera)
{
	bool exists = false;
	for (int i = 0; i < this->camera.size(); i++)
	{
		if (this->camera[i].source == camera.source)
			exists = true;		
	}
	if (!camera.takeFrame())
		return false;
	if (exists) 
	{
		std::cout << "This camera" << camera.source << " is already added or is not valid!" << endl;
		return false;
	}
	else
	{
		this->camera.push_back(camera);
		//this->camera.push_back(Camera(camera.source));
		return true;
	}	
}

bool AllCameras::setColorValues(Controllers& controllers , int which , string port)
{
	if (which == 0)
		std::cout << "Setting the color for Right Controller." << endl;
	else if (which == 1)
		std::cout << "Setting the color for Left Controller." << endl;
	else if (which == 2)
		std::cout << "Setting the color for Head Tracker." << endl;

	int color[3][2] = { { 0 , 255 } ,{ 0 , 255 }  ,{ 0 , 255 } };
	namedWindow("Color", CV_WINDOW_AUTOSIZE); //create a window called "Color"
	//Create trackbars in "Color" window
	cvCreateTrackbar("LowH", "Color", &color[0][0], 255); //Hue (0 - 255)
	cvCreateTrackbar("HighH", "Color", &color[0][1], 255);
	cvCreateTrackbar("LowS", "Color", &color[1][0], 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Color", &color[1][1], 255);
	cvCreateTrackbar("LowV", "Color", &color[2][0], 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Color", &color[2][1], 255);

	while (true)
	{
		for (int i = 0; i < this->camera.size(); i++)
		{
			this->camera[i].displayMask = true;
			if (this->camera[i].takeFrame())
			{
				this->camera[i].takeMaskFrame(color);
				this->camera[i].showFrame();
				this->camera[i].showMask();
			}
		}
		char c = waitKey(10);
		if (c == 13)
		{
			for (int i = 0; i < this->camera.size(); i++)
			{
				this->camera[i].displayMask = false;
				this->camera[i].showMask();
			}
			Controller::setColor(color, controllers, which , port);
			controllers.addedControllers[which] == true;
			destroyWindow("Color");
			return true;
		}
		else if (c == 'q' || c == 'Q' || c == 27)
		{
			std::cout << "Cancel." << endl;
			for (int i = 0; i < this->camera.size(); i++)
			{
				this->camera[i].displayMask = false;
				this->camera[i].showMask();
			}
			destroyWindow("Color");
			return false;
		}
	}
}

void AllCameras::trackObject(Controllers& controllers)
{
	if (this->debug)
		cout << "[DEBUG:All cameras tracks all controllers]" << endl;
	vector<vector<bool>> found(3);
	for (int i = 0; i < this->camera.size(); i++)
		this->camera[i].takeFrame();

	for (int con = 0; con < 3; con++)
	{
		if (controllers.addedControllers[con] == true)
		{
			controllers.serialPorts[con]->clearBuffer();

			if (this->debug)
				if (con == 0)
					cout << "[DEBUG:Tracking Right controller]" << endl;
				else if (con == 1)
					cout << "[DEBUG:Tracking Left controller]" << endl;
				else if (con == 2)
					cout << "[DEBUG:Tracking Head Tracker]" << endl;

			int numOfFound = 0;
			int camerasFoundID[2];
			bool fastWay = false;
			vector<Point2f> imgPoint0;
			vector<Point2f> imgPoint1;

			for (int cam = 0; cam < this->camera.size(); cam++)
			{
				if (this->camera[cam].trackObject(controllers.color[con], con , fastWay))
					if (numOfFound == 0)
					{
						fastWay = true;
						numOfFound++;
						imgPoint0.push_back(this->camera[cam].center);
						camerasFoundID[0] = cam;
					}
					else
					{
						numOfFound++;
						imgPoint1.push_back(this->camera[cam].center);
						camerasFoundID[1] = cam;
					}
			}
			if (numOfFound == 2)
			{
				if (this->debug)
					cout << "[DEBUG:Trialngulate Points!]" << endl;
				Mat pnts3D(4, 1, CV_64F);
				triangulatePoints(this->camera[camerasFoundID[0]].projectionMatrix, this->camera[camerasFoundID[1]].projectionMatrix, imgPoint0, imgPoint1, pnts3D);
				Controller::setPosition(controllers , con , pnts3D);
			}
			else if (numOfFound > 0)
				Controller::setPosition(controllers , con , this->camera[camerasFoundID[0]].position, false);

			Controller::getSerialData(controllers, con);
		}
		
	}

	for (int cam = 0; cam < this->camera.size(); cam++)
		this->camera[cam].showFrame();
}

void AllCameras::showCamerasFrames(bool show)
{
	for (int i = 0; i < this->camera.size(); i++)
	{
		this->camera[i].displayFrame = show;
		this->camera[i].takeFrame();
		this->camera[i].showFrame();
	}
}

void AllCameras::setDisplayFrames(bool show)
{
	for (int i = 0; i < this->camera.size(); i++)
		this->camera[i].displayFrame = show;
}

void AllCameras::findCamerasPosition(int color[][2])
{
	Camera::displayFrame = true;
	int numOfCaptures = 0;
	while (numOfCaptures < 5)
	{
		bool capture = false;
		if (waitKey(5) == 13)
		{
			capture = true;
			numOfCaptures++;
			std::cout << ">Captured position " << numOfCaptures << " ." << endl;
		}
		for (int i = 0; i < this->camera.size(); i++)
		{
			this->camera[i].takeFrame();
			this->camera[i].captureObject(color, capture);
		}
	}
	vector<Point3f> objectPoints;
	objectPoints.push_back(Point3f(-9.8, -7.4, 0.));
	objectPoints.push_back(Point3f(-9.9, 8.1, 0.));
	objectPoints.push_back(Point3f(0., 0., 0.));
	objectPoints.push_back(Point3f(10.0, 8.0, 0.));
	objectPoints.push_back(Point3f(9.7, -7.7, -0.));
	for (int i = 0; i < this->camera.size(); i++)
		this->camera[i].findCameraPosition(objectPoints);
}