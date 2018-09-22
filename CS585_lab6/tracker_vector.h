#include "tracker.h"
#include "HungarianAlg.h"
#pragma once
using namespace std;
using namespace cv;
class carsTrack
{
	public:
		carsTrack(float dT, float dThres = 50, int mSKA = 10, int tLength = 40);
		~carsTrack(void);
		vector<carTrack> carTrackVector;
		void setCarsTrack(float dT, float dThres = 50, int mSKA = 10, int tLength = 40);
		void carsTrackUpdate(vector<Point>& detectPoints, Point updateAreaPtLT, Point updateAreaPtRB);
		void changeAllMyKF(float delta_time, float pc, float mc, float ec);
	private:
		float delta_time;
		float distanceThres;
		float distanceThres2;
		int maxSkippedFramesAllowed;
		int trace_length;
		int nextcarTrackID;
};
