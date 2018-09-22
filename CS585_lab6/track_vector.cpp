#include"tracker_vector.h"
#include <memory>

carsTrack::carsTrack(float dT, float dThres, int mSKA, int tLength) :delta_time(dT), distanceThres(dThres), maxSkippedFramesAllowed(mSKA), trace_length(tLength), nextcarTrackID(0)
{
}

carsTrack::~carsTrack(void)
{
}

void carsTrack::changeAllMyKF(float delta_time, float pc, float mc, float ec)
{
	for (int i = 0; i < this->carTrackVector.size(); i++)
	{
		carTrackVector[i].KF.setMyKalmanFilter(delta_time, pc, mc, ec);
	}
	return;
}

void carsTrack::setCarsTrack(float dT, float dThres, int mSKA, int tLength)
{
	this->delta_time = dT;
	this->distanceThres = dThres;
	this->maxSkippedFramesAllowed = mSKA;
	this->trace_length = tLength;
	return;
}

bool isInarea(Point areaPtLT, Point areaPtRB, Point pt)
{
	if (pt.x<areaPtLT.x || pt.y<areaPtLT.y || pt.x>areaPtRB.x || pt.y>areaPtRB.y)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void carsTrack::carsTrackUpdate(vector<Point>& detectPoints, Point updateAreaPtLT, Point updateAreaPtRB)
{
	if (!carTrackVector.size())
	{
		for (int i = 0; i < detectPoints.size(); i++)
		{
			carTrack temp = carTrack(detectPoints[i], delta_time, nextcarTrackID++, 0.5, 0.5, 0.1);
			//carTrack::carTrack(Point& p, float delta_time, int trackID, float pc, float mc, float ec)
			carTrackVector.push_back(temp);
		}
	}
	int N = carTrackVector.size();
	int M = detectPoints.size();

	vector<int> assignmentMatrix;
	vector<float> Cost(N*M);
	if (carTrackVector.size() != 0)
	{

		for (int i = 0; i < N; i++)
		for (int j = 0; j < M; j++)
		{
			float d = carTrackVector[i].distance(detectPoints[j]);
			Cost[i + j*N] = d;

		}
	}

	AssignmentProblemSolver APS;
	APS.Solve(Cost, N, M, assignmentMatrix, AssignmentProblemSolver::optimal);
	for (int i = 0; i < assignmentMatrix.size(); i++)
	{
		if (assignmentMatrix[i] != -1)
		{
			if (Cost[i + assignmentMatrix[i] * N] > distanceThres)
			{

				assignmentMatrix[i] = -1;
				carTrackVector[i].framesSkipped = 1;
			}
			else if (Cost[i + assignmentMatrix[i] * N] < 10)
			{
				//assignmentMatrix[i] = -1;
				carTrackVector[i].framesSkipped++;
			}
		}
		else
		{
			carTrackVector[i].framesSkipped++;
		}

	}



	for (int i = 0; i < carTrackVector.size(); i++)
	{
		if (carTrackVector[i].framesSkipped > maxSkippedFramesAllowed)
		{
			carTrackVector.erase(carTrackVector.begin() + i);
			assignmentMatrix.erase(assignmentMatrix.begin() + i);
			i--;
		}
	}

	for (size_t i = 0; i < detectPoints.size(); ++i)
	{
		if (find(assignmentMatrix.begin(), assignmentMatrix.end(), i) == assignmentMatrix.end())
		{
			if (isInarea(updateAreaPtLT, updateAreaPtRB, detectPoints[i]))
			{
				carTrackVector.push_back(carTrack(detectPoints[i], delta_time, nextcarTrackID++, 0.5, 0.5, 0.1));
			}
		}
	}

	for (int i = 0; i<assignmentMatrix.size(); i++)
	{

		if (assignmentMatrix[i] != -1)
		{
			carTrackVector[i].framesSkipped = 0;
			carTrackVector[i].carTrackUpdate(detectPoints[assignmentMatrix[i]], true, trace_length);
		}
		else
		{
			carTrackVector[i].carTrackUpdate(Point(), false, trace_length);
		}
	}

}

