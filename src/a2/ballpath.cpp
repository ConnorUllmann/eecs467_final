
#include <iostream>
#include <math/point.hpp>
#include <vector>
#include "CoordinateConverter.hpp"
using namespace std;


double WIDTH = 0.5;
double TIME_DIFF = 0.4; //Time difference between two frames (seconds).
double PREDICTION_TIME = 5;
double ARM_LENGTH = 0.2;


void predictPath(double x1, double y1, double x2, double y2)
{
	double vx = (x2-x1)/TIME_DIFF;
	double vy = (y2-y1)/TIME_DIFF;
	double time = 0;
	double xt = x2;
	double yt = y2;
	while(time<PREDICTION_TIME && ((xt*xt + yt*yt) > (ARM_LENGTH * ARM_LENGTH)))
	{
		time += TIME_DIFF;
		xt   += vx * TIME_DIFF;
		yt   += vy * TIME_DIFF;
		if(xt > WIDTH/2)
		{
			vx = -vx;
			xt = WIDTH-xt;
		}
		if(xt < -WIDTH/2)
		{
			vx = -vx;
			xt = -WIDTH - xt;
		}
		cout<<"("<<xt<<", "<<yt<<")"<<endl;
	}

	if((xt* xt) + (yt*yt) <= (ARM_LENGTH * ARM_LENGTH))
	{
		cout<<"Can block ball at ("<<xt<<", "<<yt<<")"<<endl;
	}
};

//Estimates from a set of positions using acceleration + velocity. Returns a set of positions predicted out PREDICTION_TIME seconds.
vector<eecs467::Point<double>> predictPath2(vector<BlobDetector::Blob>& pos)
{
    vector<eecs467::Point<double>> ret;
    if(pos.size() < 3) return ret;

    vector<eecs467::Point<double>> v; //size = pos.size() - 1
    vector<eecs467::Point<double>> a; //size = pos.size() - 2
    for(int i = 1; i < pos.size(); i++)
    {
        eecs467::Point<double> p((pos[i].x - pos[i-1].x)/TIME_DIFF, (pos[i].y - pos[i-1].y)/TIME_DIFF);
        v.push_back(p);
    }
    for(int i = 1; i < v.size(); i++)
    {
        eecs467::Point<double> p((v[i].x - v[i-1].x)/TIME_DIFF, (v[i].y - v[i-1].y)/TIME_DIFF);
        a.push_back(p);
    }

    eecs467::Point<double> aAvg(0, 0); //The average acceleration we will predict with.
    for(int i = 0; i < a.size(); i++)
    {
        aAvg.x += a[i].x;
        aAvg.y += a[i].y;
    }
    aAvg.x /= a.size();
    aAvg.y /= a.size();

    eecs467::Point<double> vel(v[v.size()-1]); //The starting velocity we will predict with.
    eecs467::Point<double> p(pos[pos.size()-1].x, pos[pos.size()-1].y); //The starting position we will predict from.




    double t = 0;
    while(t < PREDICTION_TIME)
    {
        vel.x += aAvg.x * TIME_DIFF;
        vel.y += aAvg.y * TIME_DIFF;

        p.x += vel.x * TIME_DIFF;
        p.y += vel.y * TIME_DIFF;

        // std::array<int, 2> arr{{p.x, p.y}};

        ret.push_back(p);
        t += TIME_DIFF;
    }

    return ret;
}

/*int main()
{
	double x1, x2, y1, y2;
	cout<<"x1: ";
	cin>>x1;
	cout<<"y1: ";
	cin>>y1;
	cout<<"x2: ";
	cin>>x2;
	cout<<"y2: ";
	cin>>y2;		
	predictPath(x1, y1, x2, y2);
	return 0;
}*/
