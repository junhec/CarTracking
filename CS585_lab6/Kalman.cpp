#include "Kalman.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace cv;

void MyKalmanFilter::setMyKalmanFilter(float dt, float pc, float mc, float ec)
{
	this->deltatime = dt;
	this->pc = pc;
	this->mc = mc;
	this->ec = ec;
}

MyKalmanFilter::MyKalmanFilter(Point_<float> p, float dt, float pc, float mc, float ec) :pc(pc), mc(mc), ec(ec)
{
	
	deltatime = dt;
	KF = new cv::KalmanFilter(4,2,0);
	//KF.init(4, 2, 0);

	Point_<float> LastResult;
	KF->statePre.setTo(0);
	KF->statePre.at<float>(0, 0) = p.x;
	KF->statePre.at<float>(1, 0) = p.y;

	KF->statePost.setTo(0);
	KF->statePost.at<float>(0, 0) = p.x;
	KF->statePost.at<float>(1, 0) = p.y;

	KF->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, deltatime, 0, 0, 1, 0, deltatime, 0, 0, 1, 0, 0, 0, 0, 1);
	setIdentity(KF->measurementMatrix);
	KF->processNoiseCov = (cv::Mat_<float>(4, 4) <<
		pow(deltatime, 2.0) / 4.0, 0, pow(deltatime, 3.0) / 2.0, 0,
		0, pow(deltatime, 2.0) / 4.0, 0, pow(deltatime, 3.0) / 2.0,
		pow(deltatime, 3.0) / 2.0, 0, pow(deltatime, 2.0), 0,
		0, pow(deltatime, 3.0) / 2.0, 0, pow(deltatime, 2.0));
	KF->processNoiseCov*=this->pc;
	setIdentity(KF->measurementNoiseCov, Scalar::all(this->pc));
	setIdentity(KF->errorCovPost, Scalar::all(this->pc));
}

Point_<float> MyKalmanFilter::kalmanPredict()
{
	Mat prediction = KF->predict();
	LastResult = Point_<float>(prediction.at<float>(0), prediction.at<float>(1));

	KF->statePre.copyTo(KF->statePost);
	KF->errorCovPre.copyTo(KF->errorCovPost);

	return LastResult;
}

Point_<float> MyKalmanFilter::kalmanCorrect(Point_<float> p, bool DataCorrect)
{
	Mat measurement(2, 1, CV_32FC(1));
	if (!DataCorrect){		
		measurement.at<float>(0) = LastResult.x;//Update with prediction
		measurement.at<float>(1) = LastResult.y;
	}
	else{
		measurement.at<float>(0) = p.x;//Update with measurement
		measurement.at<float>(1) = p.y;
	}
	Mat estimated = KF->correct(measurement);
	LastResult = Point_<float>(estimated.at<float>(0), estimated.at<float>(1));
	return LastResult;
}

MyKalmanFilter::~MyKalmanFilter(){
}