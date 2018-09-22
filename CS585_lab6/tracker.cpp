#include"tracker.h"
using namespace cv;
using namespace std;

carTrack::carTrack(Point& p, float delta_time, int trackID,float pc, float mc, float ec):
carTrackID(trackID),
framesSkipped(0),
prediction(p),
KF(p, delta_time,pc,mc,ec)
{
}

//MyKalmanFilter::MyKalmanFilter(Point_<float> p, float dt = 10, float pc, float mc, float ec)

float carTrack::distance(Point& pt)
		{
			Point temp = prediction - pt;
			float result = sqrtf(float(temp.x)*float(temp.x) + float(temp.y)*float(temp.y));
			return result;
		}

void carTrack::carTrackUpdate(Point& pt, bool isDetected, int traceLength)
		{
			KF.kalmanPredict();
			prediction = KF.kalmanCorrect(pt, isDetected);
			if (carPointTrace.size() > traceLength)
				{
					carPointTrace.erase(carPointTrace.begin(), carPointTrace.end() - traceLength);
				}
					carPointTrace.push_back(prediction);
		}

