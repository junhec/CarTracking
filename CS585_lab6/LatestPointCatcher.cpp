#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include "tracker_vector.h"
#pragma once
using namespace cv;
using namespace std;

vector<Point> carPointTraceToVectorPoint(carTrack ct)
{
	return ct.carPointTrace;
}
vector<vector<Point>> carTrackVectorToVectorVectorPoint(carsTrack cst)
{
	vector<vector<Point>> res;
	for (int i = 0; i < cst.carTrackVector.size(); i++)
	{
		res.push_back(carPointTraceToVectorPoint(cst.carTrackVector[i]));
	}
	return res;
}
vector<Point> getLastPointSet(carsTrack cst)
{
	vector<Point> res;
	vector<vector<Point>> vvp = carTrackVectorToVectorVectorPoint(cst);
	for (int i = 0; i < vvp.size(); i++)
	{
		res.push_back(vvp[i].at(vvp[i].size()-1));
	}
	return res;
}