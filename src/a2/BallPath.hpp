#ifndef BALL_PATH_HPP
#define BALL_PATH_HPP

#include <iostream>
#include <math/point.hpp>
#include <vector>
#include <deque>
#include "CoordinateConverter.hpp"
#include "a2/Arm.hpp"
#include "BlobDetector.hpp"
#include "CalibrationHandler.hpp"

using namespace std;

class BallPath
{
    static BallPath* _instance;
public:
    static BallPath* instance();
    double TIME_DIFF; //Time difference between two frames (seconds).
    double WIDTH;
    double MAX_PREDICTION_TIME;
    double ARM_LENGTH;
    int MAX_PREDICTION_POINT;

    BallPath() : TIME_DIFF(0.0555555), WIDTH(0.5), MAX_PREDICTION_TIME(2), ARM_LENGTH(0.2), MAX_PREDICTION_POINT(100) {};

    eecs467::Point<double> reflect(double point_x, double point_y, 
                      double normal_x, double normal_y, 
                      double normal_vec_x, double normal_vec_y);
    double dot(double v1x, double v1y, double v2x, double v2y);
    eecs467::Point<double> intersects(double Ax, double Ay, double Bx, double By, double Ex, double Ey, double Fx, double Fy, bool as_seg=true);

    void predictPath(double x1, double y1, double x2, double y2);

    //Estimates from a set of positions using acceleration + velocity. Returns a set of positions predicted out PREDICTION_TIME seconds.
    //border_x is the line that the robot arm moves along in global coordinates.
    vector<eecs467::Point<double>> predictPath2(deque<BlobDetector::Blob> pos, double border_x);
};

#endif