#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class MyKalmanFilter
{
public:
	KalmanFilter* KF;
	float deltatime;
	Point_<float> LastResult;
	void setMyKalmanFilter(float dt,float pc, float mc, float ec);
	MyKalmanFilter(Point_<float> p, float dt, float pc, float mc, float ec);
	~MyKalmanFilter();
	Point_<float> kalmanPredict();
	Point_<float> kalmanCorrect(Point_<float> p, bool DataCorrect);
private:
	float pc;
	float mc;
	float ec;
};
