#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
vector<Point> getCornerPointsByMat(Mat, int, int, double, int);
double distanceBetweenTwoPoints(Point, Point);
void contractPointsCloseToEachOther(vector<Point>&, int thres);
vector<Point>getPredictedPointSetbyTwoClosePoints(Mat&, vector<Point>, vector<Point>);

