#include"tracker.h"
#include"tracker_vector.h"
#include<math.h>
#define PI 3.14159265
int calVelocity(carsTrack cst, int ID, double angel1, double angel2, double h, double f)
{
	float tempVel = 0;
	int vel = 0;
	for (int i = 0; i < cst.carTrackVector.size(); i++)
	{
		if (cst.carTrackVector[i].carTrackID == ID)
		{
			if (cst.carTrackVector.at(i).carPointTrace.size()<2)
			{
				/*int s = cst.carTrackVector[i].carPointTrace.size();
				Point start = cst.carTrackVector.at(i).carPointTrace.at(0);
				Point end = cst.carTrackVector.at(i).carPointTrace.at(cst.carTrackVector.at(i).carPointTrace.size() - 1);
				Point temp = end - start;
				vel = abs(2*temp.y* 250/ (18 * s));
				vel = (sqrtf(float(temp.x)*float(temp.x) + float(temp.y)*float(temp.y))) / s;*/
			}
			else
			{
				int s = cst.carTrackVector[i].carPointTrace.size();

				Point pre = cst.carTrackVector.at(i).carPointTrace.at(cst.carTrackVector.at(i).carPointTrace.size() - 2);
				Point pos = cst.carTrackVector.at(i).carPointTrace.at(cst.carTrackVector.at(i).carPointTrace.size() - 1);
				Point temp = pos - pre;
				//vel = abs(25*(pos.y-pre.y)*100000/(pos.y*pre.y));
				//vel = 10*(sqrtf(float(temp.x)*float(temp.x) + float(temp.y)*float(temp.y)))*(1000/pos.y);

				temp.x = (-(temp.y*sin(angel1*PI / 180) + temp.x*cos(angel1*PI / 180)*sin(angel2*PI / 180))*h) / (sin(angel2*PI / 180)*(temp.y*cos(angel2*PI / 180) + f*sin(angel2*PI / 180)));
				temp.y = (-(temp.y*cos(angel1*PI / 180) + temp.x*sin(angel1*PI / 180)*sin(angel2*PI / 180))*h) / (sin(angel2*PI / 180)*(temp.y*cos(angel2*PI / 180) + f*sin(angel2*PI / 180)));
				tempVel = abs((sqrtf(float(temp.x)*float(temp.x) + float(temp.y)*float(temp.y))));
				vel = tempVel;
			}

		}

	}

	return vel;
}