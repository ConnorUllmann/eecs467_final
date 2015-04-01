#include <iostream>
using namespace std;


double WIDTH = 0.5;
double TIME_DIFF = 0.4;
double TIME_INTERVAL = 0.2;
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
		time += TIME_INTERVAL;
		xt   += vx * TIME_INTERVAL;
		yt   += vy * TIME_INTERVAL;
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

int main()
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
}
