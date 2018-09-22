#include"tracker_vector.h"
#include"CornerDetection.h"
#include"testingFunctions.h"
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}


int main(int argc, char** argv)
{
	VideoCapture cap("IMG_0592.mp4");
	Mat frame;
	carsTrack left_tracker(float(0.1));
	carsTrack right_tracker(float(0.1));

	namedWindow("setting");
	namedWindow("frame");
	int set1 = 2;
	int set2 = 3;
	int set3 = 2;
	int set4 = 115;

	//createTrackbar("set1", "setting", &set1, 10);
	//createTrackbar("set2", "setting", &set2, 10);
	//createTrackbar("set3", "setting", &set3, 10);
	//createTrackbar("set4", "setting", &set4, 255);
	int set5 = 2;
	int set6 = 30;
	int set7 = 360;
	int set8 = 60;
	createTrackbar("set5", "setting", &set5, 30);
	createTrackbar("set6", "setting", &set6, 60);
	createTrackbar("set7", "setting", &set7, 1000);
	createTrackbar("set8", "setting", &set8, 100);
	setMouseCallback("frame", CallBackFunc, NULL);
	int frameCount = 0;
	while (frameCount<530)
	{

		cap >> frame;
		left_tracker.setCarsTrack(double(2) / 15, double(40), 10, 20);
		right_tracker.setCarsTrack(double(2) / 15, double(40), 10, 20);
		Mat frame_gray;
		cvtColor(frame, frame_gray, CV_BGR2GRAY);
		vector<Point> detectPoints = getCornerPointsByMat(frame_gray, set1, set2, double(set3) / 100, set4);
		contractPointsCloseToEachOther(detectPoints, 50);
		vector<Point> left_pts =eraseEdgePoints(detectPoints,0,160,390,350);
		vector<Point> right_pts = eraseEdgePoints(detectPoints, 390, 160, frame.cols - 10, 350);
		left_tracker.carsTrackUpdate(left_pts,Point(0,160),Point(639,200));
		right_tracker.carsTrackUpdate(right_pts,Point(0,310),Point(639,350));
		drawTraces(frame, left_tracker,0);
		drawTraces(frame, right_tracker,1);
		drawMark(frame, left_tracker, -4, 30, 500, 50);
		drawMark(frame, right_tracker, -set5, set6, set7, set8);
		showNumberOfCars(frame, left_tracker,Point(0,72));
		imshow("frame", frame);
		frameCount++;
/*		string path_name="carTracking";
		char str[1024];
		memset(str, 0, sizeof(str));
		sprintf(str, "%03d", frameCount);
		path_name += str;
		path_name += ".jpg";

		imwrite(path_name, frame);
*/
		waitKey(1);


	}
}