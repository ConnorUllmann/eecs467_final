#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP
#include <math.h>
#include <string>

///////////////////////////////
// PHYSICAL CONSTANTS
///////////////////////////////

// arm lengths are in meters
const float ARM_LENGTH_0 = 0.116;
const float ARM_LENGTH_1 = 0.1;
const float ARM_LENGTH_2 = 0.1;
const float ARM_LENGTH_3 = 0.178; // including claw
const float CLAW_LENGTH = 0.082; // from claw joint to tip
const float ARM_ANGLE_MIN[] = {-3.14,-2.094,-2.094,-2.094,-2.618,-0.9 };
const float ARM_ANGLE_MAX[] = { 3.14, 2.094, 2.094, 2.094, 2.618, 2.3 };

const float CLAW_OPEN_ANGLE = 0.7;
const float CLAW_CLOSED_ANGLE = 1.65;

const float ARM_SPEED = 0.06;
const float ARM_MAX_TORQUE = 0.9;
const std::string ARM_CHANNEL_NAME = "ARM_COMMAND";
const float ARM_ERROR_THRESH = 0.05;
const float HAND_ERROR_THRESH = 0.2;
const float BALL_ANGULAR_OFFSET = 10.0 * M_PI / 180;
const float BALL_RADIAL_OFFSET = 0.02;

const std::string colorCalibrationFileName = "color_calib.txt";
const std::string maskCalibFileName = "mask_calib.txt";
const std::string transformCalibFileName = "transform_calib.txt";

//////////////////////////////
// BLOB DETECTION CONSTANTS
//////////////////////////////
const float noObjectThresholdVal = 0.4;
const float noObjectThresholdSat = 0.4;
const double objectHueRedHalfRange = 5.0;
const double objectHueGreenHalfRange = 15.0;
const double objectHueBlueHalfRange = 15.0;
const double RAD = M_PI / 180;
const float blobMinPixels = 100;


// (x, y) global coordinates of blue corners in meters
const float centerToCenterDistance = 0.24;
const float cornerTopLeftLoc[] = {0, centerToCenterDistance/2};
const float cornerTopRightLoc[] = {centerToCenterDistance, centerToCenterDistance/2};
const float cornerBottomLeftLoc[] = {0, -centerToCenterDistance/2};
const float cornerBottomRightLoc[] = {centerToCenterDistance, -centerToCenterDistance/2};
 
// the length of one side of the board in meters.
const float boardSideLength = 0.18;
const float boardDiagonalLength = 0.34; // center of blue to center of blue
const float blueSquareSideLength = 0.06; // meters
const float blueSquareDiagonalLength = 0.08202438; // meters

#endif /* CONSTANTS_HPP */
