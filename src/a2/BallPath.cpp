#include "BallPath.hpp"

using namespace std;


BallPath* BallPath::_instance = new BallPath();
BallPath* BallPath::instance() {
    return _instance;
}

void BallPath::predictPath(double x1, double y1, double x2, double y2)
{
	double vx = (x2-x1)/TIME_DIFF;
	double vy = (y2-y1)/TIME_DIFF;
	double time = 0;
	double xt = x2;
	double yt = y2;
	while(time<MAX_PREDICTION_TIME && ((xt*xt + yt*yt) > (ARM_LENGTH * ARM_LENGTH)))
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
//border_x is the line that the robot arm moves along in global coordinates.
vector<eecs467::Point<double>> BallPath::predictPath2(deque<BlobDetector::Blob> pos, double border_x)
{
    vector<eecs467::Point<double>> ret;

    if (pos.size() == 0) {
        return ret;
    }
    if (pos.size() == 1) {
        ret.push_back(eecs467::Point<double>(pos[0].x, pos[0].y));
        return ret;
    }
    // else if (pos.size() == 2) {
    //     ret.push_back(eecs467::Point<double>(pos[1].x, pos[1].y));
    //     ret.push_back(eecs467::Point<double>(pos[0].x, pos[0].y));
    //     return ret;
    // }
    // if(pos.size() < 3) return ret;

    vector<eecs467::Point<double>> v; //size = pos.size() - 1
    vector<eecs467::Point<double>> a; //size = pos.size() - 2
    for(int i = 1; i < pos.size(); i++)
    {
        // double time_diff = (pos[i].utime - pos[i-1].utime)/1000000.0;// cout << "t[" << i << "] " << time_diff << "," << pos[i].utime << "," << pos[i-1].utime << endl;

        eecs467::Point<double> p(pos[i].x - pos[i-1].x, pos[i].y - pos[i-1].y);
        v.push_back(p);
    }

    eecs467::Point<double> vAvg(0, 0);
    int NUM_TRACKED = 3;
    if (pos.size() == 2) {
        NUM_TRACKED = 2;
    }


    int start = v.size() - NUM_TRACKED;
    int i = (start > 0 ? start : 0);
    int num = 0;

    double difftime = 0.055555555;
    difftime = (pos[pos.size()-1].utime - pos[i].utime)/1000000.0;
    
    for(; i < v.size(); i++)
    {
        vAvg.x += v[i].x;
        vAvg.y += v[i].y;
        num++;

// std::cout << "v[" << i << "] " << (v[i].x*v[i].x+v[i].y+v[i].y) << std::endl;
    }
    vAvg.x /= abs(difftime);
    vAvg.y /= abs(difftime);
    /*for(int i = 1; i < v.size(); i++)
    {
        double time_diff = 1.0*(pos[i].utime - pos[i-1].utime)/1000000.0;
        eecs467::Point<double> p(1.0*(v[i].x - v[i-1].x)/time_diff, 1.0*(v[i].y - v[i-1].y)/time_diff);
        a.push_back(pv);
    }

    eecs467::Point<double> aAvg(0, 0); //The average acceleration we will predict with.
    int NUM_TRACKED = 10;
    int start = a.size() - NUM_TRACKED;instance
    int i = (start > 0 ? start : 0);
    int num = 0;
    for(; i < a.size(); i++)
    {
        int m = i - start + 1;
        aAvg.x += a[i].x * m;
        aAvg.y += a[i].y * m;
        num += m;
    }
    aAvg.x /= num;
    aAvg.y /= num;*/

    // eecs467::Point<double> vel(v[v.size()-1]); //The starting velocity we will predict with.
    eecs467::Point<double> p(pos[pos.size()-1].x, pos[pos.size()-1].y); //The starting position we will predict from.
    CoordinateConverter c;

    ret.push_back(p);

    double t = 0;
    while(t < MAX_PREDICTION_TIME && ret.size() < MAX_PREDICTION_POINT)
    {
        //vel.x += aAvg.x * TIME_DIFF;
        //vel.y += aAvg.y * TIME_DIFF;

        eecs467::Point<double> vEnd(p.x + vAvg.x * TIME_DIFF, p.y + vAvg.y * TIME_DIFF);

        auto walls = CalibrationHandler::instance()->getWalls();
        for(int i = 0; i < walls.size(); i+=2)
        {
            auto p1 = walls[i];
            auto p2 = walls[i+1];
            auto intersection = intersects(p1.x, p1.y, p2.x, p2.y, p.x, p.y, vEnd.x, vEnd.y, true);
            if(intersection.x != -1 || intersection.y != -1)
            {
                //We've intersected our velocity vector with a wall.
                eecs467::Point<double> wallLine(p2.x - p1.x, p2.y - p1.y);
                eecs467::Point<double> vEndNew = reflect(vEnd.x, vEnd.y, p1.x, p1.y, wallLine.x, wallLine.y);
                
                eecs467::Point<double> pNew = reflect(p.x, p.y, p1.x, p1.y, wallLine.x, wallLine.y);
                vAvg.x = (vEndNew.x - pNew.x)/TIME_DIFF;
                vAvg.y = (vEndNew.y - pNew.y)/TIME_DIFF;
                vEnd.x = vEndNew.x;
                vEnd.y = vEndNew.y;
            }
        }

        p.x = vEnd.x;
        p.y = vEnd.y;
        //p.x += vAvg.x * TIME_DIFF;
        //p.y += vAvg.y * TIME_DIFF;

        ret.push_back(p);
        t += TIME_DIFF;

        // if(vAvg.x >= 0)// && aAvg.x >= 0)
        //     break;

        std::array<int, 2> arr{{(int)p.x, (int)p.y}};
        std::array<float, 2> arr2 = c.imageToGlobal(arr);
        if(arr2[0] <= border_x)//arr2[0]*arr2[0] + arr2[1]*arr2[1] <= ARM_LENGTH*ARM_LENGTH)
        {
            //cout<<"catching ball at ("<<arr2[0]<<", "<<arr2[1]<<")"<<endl;
            //Arm::instance()->addCommandMovePoint(arr2[0], arr2[1]);
            break;
        }
    }

    // ret.clear();

    // ret.push_back(eecs467::Point<double>(pos[pos.size()-1].x, pos[pos.size()-1].y));

// cout << "y " << pos[pos.size()-1].y << endl;

    return ret;
}

eecs467::Point<double> BallPath::reflect(double point_x, double point_y, 
                      double normal_x, double normal_y, 
                      double normal_vec_x, double normal_vec_y)
{
    double normal_vec_length = sqrt(normal_vec_x * normal_vec_x + normal_vec_y*normal_vec_y);
    normal_vec_x /= normal_vec_length;
    normal_vec_y /= normal_vec_length;
    
    double n_point_x = point_x - normal_x;
    double n_point_y = point_y - normal_y;
    double n_length = 2 * dot(n_point_x, n_point_y, normal_vec_x, normal_vec_y);

    Point<double> ret(-point_x + 2*normal_x + normal_vec_x*n_length, 
                      -point_y + 2*normal_y + normal_vec_y*n_length);
    return ret;
}

double BallPath::dot(double v1x, double v1y, double v2x, double v2y)
{
    return v1x * v2x + v1y * v2y;
}

eecs467::Point<double> BallPath::intersects(double Ax, double Ay, double Bx, double By, double Ex, double Ey, double Fx, double Fy, bool as_seg)
{
    eecs467::Point<double> ret(-1, -1);

    double a1 = By - Ay;
    double a2 = Fy - Ey;
    double b1 = Ax - Bx;
    double b2 = Ex - Fx;
    double c1 = Bx * Ay - Ax * By;
    double c2 = Fx * Ey - Ex * Fy;

    double denom = a1 * b2 - a2 * b1;
    if(denom == 0)
        return ret;

    eecs467::Point<double> ip((b1*c2-b2*c1)/denom, (a2*c1-a1*c2)/denom);
    if(as_seg &&
       ((ip.x - Bx) * (ip.x - Bx) + (ip.y - By) * (ip.y - By) > (Ax - Bx) * (Ax - Bx) + (Ay - By) * (Ay - By) ||
        (ip.x - Ax) * (ip.x - Ax) + (ip.y - Ay) * (ip.y - Ay) > (Ax - Bx) * (Ax - Bx) + (Ay - By) * (Ay - By) ||
        (ip.x - Fx) * (ip.x - Fx) + (ip.y - Fy) * (ip.y - Fy) > (Ex - Fx) * (Ex - Fx) + (Ey - Fy) * (Ey - Fy) ||
        (ip.x - Ex) * (ip.x - Ex) + (ip.y - Ey) * (ip.y - Ey) > (Ex - Fx) * (Ex - Fx) + (Ey - Fy) * (Ey - Fy)))
            return ret;

    ret.x = ip.x;
    ret.y = ip.y;
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
