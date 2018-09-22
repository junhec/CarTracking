#include"testingFunctions.h"
#include"CornerDetection.h"
void drawCornerCirclesInMat(Mat& frame, vector<Point> pts)
{
	int i = 0;
	while (i < pts.size())
	{
		circle(frame, pts.at(i),5 , Scalar(255, 0, 0), 2, 8, 0);
		i++;
	}


}

void showNumberOfCars(Mat& frame, carsTrack cst,Point pt)
{
	int fontface = FONT_HERSHEY_SIMPLEX;
	string result = "car number: ";
	result+=to_string(cst.carTrackVector.back().carTrackID);
	putText(frame, result, pt, fontface, 0.5, Scalar(0, 0, 0));
	return;
}

void drawMark(Mat& frame, carsTrack cst, double angel1, double angel2, double h, double f)
{
	if (cst.carTrackVector.size() > 0)
	{
		for (int i = 0; i < cst.carTrackVector.size(); i++)
		{
			if (!cst.carTrackVector.at(i).carPointTrace.empty())
			{
				string mark = "";
				int tempVel = calVelocity(cst, cst.carTrackVector.at(i).carTrackID, angel1, angel2, h, f);
				mark += to_string(cst.carTrackVector.at(i).carTrackID);
				mark += " Speed: ";
				mark += to_string(tempVel);
				int fontface = FONT_HERSHEY_SIMPLEX;
				Point pt = cst.carTrackVector.at(i).carPointTrace.back();
				putText(frame, mark, pt, fontface, 0.5, Scalar(255, 255, 255));

			}
		}
	}
}

void drawLineBetweenClosePoints(Mat& frame, vector<Point> pts)
{
	int i = 0;

	while (i < pts.size())
	{
		int j = 0;
		while (j < pts.size())
		{	//distanceBetweenTwoPoints(pts.at(i), pts.at(j))>5 && 
			if (distanceBetweenTwoPoints(pts.at(i), pts.at(j)) < 10)
			{
				line(frame, pts.at(i), pts.at(j), Scalar(255), 2);
			}
			j++;
		}
		i++;

	}
}

void drawLinesBetweenPointsSetsInTwoFrames(Mat& frame, vector<Point>& pts1, vector<Point>& pts2, vector<Point>& pts3)
{
	int i = 0;
	while (i < pts1.size())
	{
		int j = 0;
		while (j < pts3.size())
		{
			if (distanceBetweenTwoPoints(pts1.at(i), pts3.at(j)) < 10)
			{
				int k = 0;
				bool isfound = false;
				while (k < pts2.size())
				{
					if (distanceBetweenTwoPoints(pts3.at(j), pts2.at(k)) < 5)
					{
						line(frame, pts1.at(i), pts2.at(k), Scalar(255), 2);
						isfound = true;
						break;
					}
					k++;

				}
				if (isfound == false)
				{
					line(frame, pts1.at(i), pts3.at(j), Scalar(255), 2);
					pts2.push_back(pts3.at(j));
				}
			}
			j++;
		}
		i++;

	}
	return;
}

vector<Point> getVectorPointFromcarTrack(carTrack ct)
{
	return ct.carPointTrace;
}

vector<vector<Point>> getVectorVectorPointFromCarsTrack(carsTrack cst)
{
	vector<vector<Point>> res;
	int i = 0;
	while (i < cst.carTrackVector.size())
	{
		res.push_back(getVectorPointFromcarTrack(cst.carTrackVector[i]));
		i++;
	}
	return res;
}
vector<Point> getCurrentPoints(carsTrack cst)
{
	vector<vector<Point>> vvp;
	vector<Point> res;
	vvp=getVectorVectorPointFromCarsTrack(cst);
	int i = 0;
	while (i < vvp.size())
	{
		if (vvp[i].size())
		{
			res.push_back(vvp[i].at(vvp[i].size() - 1));
		}
		i++;
	}
	return res;
}

void drawTrace(Mat& frame, carTrack ct)
{
	int i = 0;
	cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 155, 0), cv::Scalar(0, 0, 155), cv::Scalar(255, 245, 0), cv::Scalar(0, 255, 245), cv::Scalar(245, 0, 255), cv::Scalar(245, 127, 255), cv::Scalar(127, 0, 245), cv::Scalar(127, 0, 126) };
	while (i < ct.carPointTrace.size() - 1)
	{
		if (distanceBetweenTwoPoints(ct.carPointTrace.at(i), ct.carPointTrace.at(i + 1)) < 40)
		{		
			line(frame, ct.carPointTrace.at(i), ct.carPointTrace.at(i + 1), Colors[ct.carTrackID% 9], 3);
		}
		i++;
	}
}

void drawTraces(Mat& frame, carsTrack cst, int j)
{
	int i = 0;
	while (i < cst.carTrackVector.size())
	{
		if (cst.carTrackVector.at(i).carPointTrace.size() != 0)
		{
			
			Point start = cst.carTrackVector.at(i).carPointTrace.at(0);
			Point end = cst.carTrackVector.at(i).carPointTrace.at(cst.carTrackVector.at(i).carPointTrace.size() - 1);
			if (j == 0)
			{
				if (start.x>end.x&&start.y < end.y)
				{


					drawTrace(frame, cst.carTrackVector[i]);
				}
			}
			else
			{
				if (start.x > end.x&&start.y > end.y)
				{


					drawTrace(frame, cst.carTrackVector[i]);
				}
			}
		}
		i++;
	}
}

vector<Point> eraseEdgePoints(vector<Point> pts, int newX1, int newY1, int newX2, int newY2)
{
	int i = 0;
	while (i<pts.size())          
	{
		if (pts.at(i).x<newX1 || pts.at(i).y<newY1 || pts.at(i).x>newX2|| pts.at(i).y>newY2)
		{
			pts.erase(pts.begin() + i);
		}
		else
		{
			i++;
		}
	}
	return pts;
	
}