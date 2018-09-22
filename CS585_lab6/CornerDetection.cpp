#include"CornerDetection.h"
vector<Point> getCornerPointsByMat(Mat frame, int blockSize, int apertureSize, double k, int thres)
{
	vector<Point> cornerPointsVec;
	Mat dst, dst_norm, dst_norm_scaled;
	dst = Mat::zeros(frame.size(), CV_32FC1);
	/// Detecting corners
	cornerHarris(frame, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
	/// Normalizing
	normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(dst_norm, dst_norm_scaled);
	for (int j = 0; j < dst_norm.rows; j++)
	{
		for (int i = 0; i < dst_norm.cols; i++)
		{
			if ((int)dst_norm.at<float>(j, i) > thres)
			{
				cornerPointsVec.push_back(Point(i, j));
			}
		}
	}
	return cornerPointsVec;
}



double distanceBetweenTwoPoints(Point ptA, Point ptB)
{
	double ax = double(ptA.x);
	double ay = double(ptA.y);
	double bx = double(ptB.x);
	double by = double(ptB.y);
	double res = sqrt((ax - bx)*(ax - bx) + (ay - by)*(ay - by));
	return res;
}



void contractPointsCloseToEachOther(vector<Point> &pts, int thres)
{
	int i = 0;
	while (i < pts.size())
	{
		int j = 0;
		while (j < pts.size())
		{
			if (distanceBetweenTwoPoints(pts.at(i), pts.at(j)) < thres && i != j)
			{
				pts.at(i).x = (pts.at(i).x + pts.at(j).x) / 2;
				pts.at(i).y = (pts.at(i).y + pts.at(j).y) / 2;
				pts.erase(pts.begin() + j);
				j = 0;
			}
			else
			{
				j++;
			}

		}
		i++;
	}
}


vector<Point>getPredictedPointSetbyTwoClosePoints(Mat& frame, vector<Point> pts1, vector<Point> pts2)
{
	vector<Point>PredictedPts;
	int i = 0;
	while (i < pts1.size())
	{
		int j = 0;
		while (j < pts2.size())
		{	//distanceBetweenTwoPoints(pts.at(i), pts.at(j))>5 && 
			if (distanceBetweenTwoPoints(pts1.at(i), pts2.at(j)) < 10)
			{
				PredictedPts.push_back(Point(2 * pts2.at(j).x - pts1.at(i).x, 2 * pts2.at(j).y - pts1.at(i).y));
			}
			j++;
		}
		i++;
	}
	return PredictedPts;
}
