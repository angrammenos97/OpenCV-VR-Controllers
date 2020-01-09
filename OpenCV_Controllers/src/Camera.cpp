#include "../headers/Camera.h"
#define	to_degrees	180 / 3.14159265358979323846  /* pi */
#define to_radians	3.14159265358979323846 / 180

using namespace std;
using namespace cv;


Camera::Camera(int source , bool debug)
{
	if (this->debug)
		cout << "[DEBUG:Setting new camera with ID:" << source << " ]" << endl;
	this->source = source;
	this->cap = VideoCapture(source);
	FileStorage fs("./data/out_camera_data_" + to_string(this->source) + ".xml" , FileStorage::READ);
	if (!fs.isOpened())
		cout << "Can't find the camera calibration file.." << endl;
	else {
		fs["camera_matrix"] >> this->cameraMatrix;
		fs["distortion_coefficients"] >> this->distCoeffs;
	}
	fs.release();
}

Camera::~Camera()
{
	string windowNameFrame = "Frame " + this->source;
	string windowNameMask = "Mask " + this->source;
	destroyWindow(windowNameMask);
	destroyWindow(windowNameFrame);
	this->cap.release();
}

bool Camera::takeFrame()
{
	if (this->debug)
		cout << "[DEBUG:Taking frame from camera" << this->source << " ]" << endl;
	if (this->cap.read(this->imgOriginal))
		return true;
	else {
		cout << "Cannot take a frame from camera " << this->source << endl;
		return false;
	}
}

Size Camera::getFrameSize()
{
	return this->imgOriginal.size();
}

void Camera::takeMaskFrame(int color[3][2])
{
	if (this->debug)
		cout << "[DEBUG:Camera" << this->source << " takes Mask frame]" << endl;
	// Must take a frame (update imgOriginal) first!!
	// Convert the captured frame from BGR to HSV
	//cvtColor(this->imgOriginal, this->imgThresholded, COLOR_BGR2HSV);
	this->imgThresholded = this->imgOriginal;
	// Threshold the image
	inRange(this->imgThresholded, Scalar(color[0][0], color[1][0], color[2][0]), Scalar(color[0][1], color[1][1], color[2][1]), this->imgThresholded);
	// morphological opening (remove small objects from the foreground)
	erode(this->imgThresholded, this->imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(this->imgThresholded, this->imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	// morphological closing (fill small holes in the foreground)
	dilate(this->imgThresholded, this->imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(this->imgThresholded, this->imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
}

void Camera::showFrame()
{
	if (this->debug)
		cout << "[DEBUG:Camera" << this->source << " displaying frame]" << endl;
	if (this->displayFrame)
		imshow("Frame_" + to_string(this->source), this->imgOriginal);
	else
		destroyWindow("Frame_" + to_string(this->source));
}

void Camera::showMask()
{
	if (this->debug)
		cout << "[DEBUG:Camera" << this->source << " displaying Mask]" << endl;
	if (this->displayMask)
		imshow("Mask_" + to_string(this->source), this->imgThresholded);
	else
		destroyWindow("Mask_" + to_string(this->source));
}

bool Camera::trackObject(int color[][2], int which , bool fastWay)
{
	if (this->debug)
		if (which == 0)
			cout << "[DEBUG:Camera" << this->source << " tracks the Right controller]" << endl;
		else if (which == 1)
			cout << "[DEBUG:Camera" << this->source << " tracks the Left controller]" << endl;
		else if (which == 2)
			cout << "[DEBUG:Camera" << this->source << " tracks the Head tracker]" << endl;
	this->takeMaskFrame(color);

	vector<vector<Point>> contours;
	findContours(this->imgThresholded, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	if (contours.size() > 0)
	{
		vector<Point> max_contour;
		double max_area = 0;
		// Find the biggest area
		for (vector<vector<Point>>::iterator i = contours.begin(); i != contours.end(); i++)
			if (max_area < contourArea(*i))
				max_contour = *i;
		// Draw circle and center on frame
		minEnclosingCircle((Mat)max_contour, this->center, this->radius);
		//minEnclosingCircle((Mat)contours[0], this->center, this->radius);
		if (this->displayFrame)
		{
			Mat imgDraw = Mat::zeros(this->imgOriginal.size(), CV_8UC3);
			circle(imgDraw, this->center, 2, Scalar(255, 0, 0), 2);
			circle(imgDraw, this->center, (int)radius, Scalar(0, 255, 255), 2);
			if (which == 0)
				cv::putText(imgDraw, "Right", (this->center - Point2f(0, this->radius)), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
			else if (which == 1)
				cv::putText(imgDraw, "Left", (this->center - Point2f(0, this->radius)), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
			else if (which == 2)
				cv::putText(imgDraw, "Head", (this->center - Point2f(0, this->radius)), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
			this->imgOriginal += imgDraw;
		}
		if (this->foundCameraPosition && !fastWay)
		{
			// Find object if none of the other camera find it
			for (vector<Point2f>::iterator j = this->imagePoints.begin(); j != this->imagePoints.end(); j++)
				circle(this->imgOriginal, *j, 2, Scalar(255, 0, 0), 2);
			float height = 564.f; // 517.281f;
			float tempcenter = 1.85f; // 1.297338f;
			float hwhm = 3.65f; // 3.752844f;
			float shape = 0.485f; // 0.4762335f;
			float a = powf((radius - tempcenter) / hwhm, 2.);
			float b = powf(2., 1. / shape) - 1.;
			float c = 1. + a * b;
			float d = height / powf(c, shape);
			float dreal = d / cosf(((this->imgOriginal.rows / 20) - (this->center.y / 10)) * to_radians);

			Mat uvPoint = Mat::ones(3, 1, DataType<double>::type); //u,v,1
			uvPoint.at<double>(0, 0) = (double)this->center.x; //got this point using mouse callback
			uvPoint.at<double>(1, 0) = (double)this->center.y;
			Mat tempMat, tempMat2;
			tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
			tempMat2 = rotationMatrix.inv() * tvec;
			double s = dreal + tempMat2.at<double>(2, 0); //285 represents the height Zconst
			s /= tempMat.at<double>(2, 0);
			this->position = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
			//cout << "Camera" << this->source << " says that " << controller.getName() << " is at: " << this->position << endl;
		}
		return true;
	}
	return false;
}

void Camera::captureObject(int color[][2] , bool capture)
{
	if (this->debug)
		cout << "[DEBUG:Camera" << this->source << " captures the Right controller]" << endl;
	this->takeMaskFrame(color);

	vector<vector<Point>> contours;
	findContours(this->imgThresholded, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	if (contours.size() > 0)
	{
		vector<Point> max_contour;
		double max_area = 0;
		// Find the biggest area
		for (vector<vector<Point>>::iterator i = contours.begin(); i != contours.end(); i++)
			if (max_area < contourArea(*i))
				max_contour = *i;
		// Draw circle and center on frame
		minEnclosingCircle((Mat)max_contour, this->center, this->radius);
		//minEnclosingCircle((Mat)contours[0], this->center, this->radius);
		if (this->displayFrame)
		{
			Mat imgDraw = Mat::zeros(this->imgOriginal.size(), CV_8UC3);
			circle(imgDraw, this->center, 2, Scalar(0, 255, 0), 3);
			circle(imgDraw, this->center, (int)radius, Scalar(0, 255, 255), 2);
			cv::putText(imgDraw, "Right", (this->center - Point2f(0, this->radius)), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
			this->imgOriginal += imgDraw;
		}
	}
	this->showFrame();

	if (capture)
		this->imagePoints.push_back(this->center);
}

void Camera::findCameraPosition(vector<Point3f> objectPoints)
{
	if (this->debug)
		cout << "[DEBUG:Camera" << this->source << " calculating it's posiotion]" << endl;
	solvePnP(objectPoints, this->imagePoints, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec);
	Rodrigues(this->rvec, this->rotationMatrix);
	projectPoints(objectPoints, this->rvec, this->tvec, this->cameraMatrix, this->distCoeffs, this->imagePoints);
	this->cameraPosition = -this->rotationMatrix * this->tvec;
	this->cameraRotation = Point3f(atan2f(this->rotationMatrix.at<double>(2, 1), this->rotationMatrix.at<double>(2, 2)) * to_degrees, atan2f(-this->rotationMatrix.at<double>(2, 0), sqrtf(powf(this->rotationMatrix.at<double>(2, 1), 2) + powf(this->rotationMatrix.at<double>(2, 2), 2))) * to_degrees, atan2f(this->rotationMatrix.at<double>(1, 0), this->rotationMatrix.at<double>(0, 0)) * to_degrees);
	cout << "Camera" << this->source << " position is: " << this->cameraPosition << endl;
	cout << "Camera" << this->source << " rotation is: " << this->cameraRotation << endl;
	cout << "Camera" << this->source << " rotation matrix is: " << this->rotationMatrix << endl;
	cout << "Camera" << this->source << " translation vector is: " << this->tvec << endl;
	this->calculateProjectionMatrix();

	this->imagePoints.clear();
}

void Camera::calculateProjectionMatrix()
{
	if (this->debug)
		cout << "[DEBUG:Camera" << this->source << " calculating it's projection matrix]" << endl;
	for (int r = 0; r < this->projectionMatrix.rows; r++)
		for (int c = 0; c < this->projectionMatrix.cols - 1; c++)
		{
			this->projectionMatrix.at<double>(r,c) = this->rotationMatrix.at<double>(r, c);
			this->projectionMatrix.at<double>(r, 3) = this->tvec.at<double>(r, 0);
		}
	this->projectionMatrix = this->cameraMatrix * this->projectionMatrix;
	cout << "Camera" << this->source << " projection matrix is: " << this->projectionMatrix << endl;
}