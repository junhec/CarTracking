#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include "tracker_vector.h"
#include "calVelocity.h"
using namespace cv;
using namespace std;

void drawLineBetweenClosePoints(Mat&, vector<Point>);
void drawCornerCirclesInMat(Mat&, vector<Point>);
void drawLinesBetweenPointsSetsInTwoFrames(Mat&, vector<Point>&, vector<Point>&, vector<Point>&);
vector<Point> getCurrentPoints(carsTrack cst); 
void showNumberOfCars(Mat& frame, carsTrack cst, Point pt);
void drawTraces(Mat& frame, carsTrack cst, int i);
void drawMark(Mat& frame, carsTrack cst, double angel1, double angel2, double h, double f);
vector<Point> eraseEdgePoints(vector<Point> pts, int newX1, int newY1, int newX2, int newY2);