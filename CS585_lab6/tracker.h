#include "opencv2/video/tracking.hpp"
#include "Kalman.h"
#pragma once
using namespace std;
using namespace cv;
class carTrack
{
	public:
		vector<Point> carPointTrace;
		int carTrackID;
		int framesSkipped;
		carTrack(Point& p, float delta_time, int trackID, float pc, float mc, float ec);
	float distance(Point& pt);
	void carTrackUpdate(Point& pt, bool isDetected, int traceLength);
	MyKalmanFilter KF;
	
private:
	Point prediction;

};
