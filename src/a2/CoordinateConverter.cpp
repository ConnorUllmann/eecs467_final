#include "CoordinateConverter.hpp"
#include "CalibrationHandler.hpp"
#include "Constants.hpp"
#include <stdio.h>
#include <math.h>

std::array<float, 2> CoordinateConverter::polarToCartesian(const std::array<float, 2>& arr) {
	std::array<float, 2> ret;
	ret[0] = arr[0] * cos(arr[1]);
	ret[1] = arr[0] * sin(arr[1]);
	return ret;
}

std::array<float, 2> CoordinateConverter::cartesianToPolar(const std::array<float, 2>& arr) {
	std::array<float, 2> ret;
	ret[0] = sqrt(arr[0] * arr[0] + arr[1] * arr[1]);
	ret[1] = atan2(arr[1], arr[0]);

	return ret;
}

std::array<int, 2> CoordinateConverter::screenToImage(const std::array<float, 2>& arr) {
	std::array<int, 2> ret;

	int imageWidth = CalibrationHandler::instance()->imageWidth();
	int imageHeight = CalibrationHandler::instance()->imageHeight();

	ret[0] = (arr[0] + 0.5) * imageWidth;
	ret[1] = imageHeight - ((arr[1] + 0.5 * ((float)imageHeight / imageWidth)) * imageWidth);
	return ret;
}

std::array<float, 2> CoordinateConverter::imageToScreen(const std::array<int, 2>& arr) {
	std::array<float, 2> ret;

	int imageWidth = CalibrationHandler::instance()->imageWidth();
	int imageHeight = CalibrationHandler::instance()->imageHeight();

	ret[0] = ((float)arr[0] / imageWidth) - 0.5;
	ret[1] = ((float)(imageHeight - arr[1]) / imageWidth)
		- 0.5 * ((float)imageHeight / imageWidth);
	return ret;
}

//arr is a location in the same coordinate frame as cornerTopLoc[].
std::array<int, 2> CoordinateConverter::globalToBoard(const std::array<float, 2>& arr) {
	std::array<int, 2> ret;
	
	float a0 = CalibrationHandler::instance()->armToRightLength();
	float a1 = CalibrationHandler::instance()->armToLeftLength();
	float b = blueSquareSideLength / 2;
	ret[0] = (int)floor(3 * (arr[0] - b) / (a0 + a1 - 2*b));
	ret[1] = (int)floor(3 * (arr[1] - b + a0) / (a0 + a1 - 2*b));
	return ret;
}

std::array<float, 2> CoordinateConverter::boardToGlobal(const std::array<int, 2>& arr) {
    std::array<float, 2> ret;
	
	float a0 = CalibrationHandler::instance()->armToRightLength();
	float a1 = CalibrationHandler::instance()->armToLeftLength();
	float b = blueSquareSideLength / 2;
	
	ret[0] = (((float)arr[0] + 0.5) / 3) * (a0 + a1 - 2 * b) + b;
	ret[1] = (((float)arr[1] + 0.5) / 3) * (a0 + a1 - 2 * b) + b - a0;
	return ret;
}

std::array<float, 2> CoordinateConverter::imageToGlobal(const std::array<int, 2>& arr) {
	Matrix<float> homogenousPt(3, 1);
	homogenousPt(0) = arr[0];
	homogenousPt(1) = arr[1];
	homogenousPt(2) = 1;

	const Matrix<float>& transform =
		CalibrationHandler::instance()->getImageToBoardTransform();

	Matrix<float> imagePt = transform * homogenousPt;
	std::array<float, 2> ret{{imagePt(0), imagePt(1)}};
	return ret;
}

std::array<float, 3> CoordinateConverter::rgbToHsv(const std::array<uint8_t, 3>& rgb) {
	std::array<float, 3> frgb;
	// Convert rgb from [0,255] to [0,1]
	frgb[0] = (float) rgb[0] / 255;
	frgb[1] = (float) rgb[1] / 255;
	frgb[2] = (float) rgb[2] / 255;  
	std::array<float, 3> hsv;
	
	float minimum, maximum, delta;

	minimum = std::min( std::min( frgb[0], frgb[1]), frgb[2] );
	maximum = std::max( std::max( frgb[0], frgb[1]), frgb[2] );
	hsv[2] = maximum;				// v

	delta = maximum - minimum;

	if( maximum != 0 )
		hsv[1] = delta / maximum;		// s
	else {
		// r = g = b = 0		// s = 0, v is undefined
		hsv[1] = 0;
		hsv[0] = -1;
		return hsv;
	}

	if( frgb[0] == maximum )
		hsv[0] = ( frgb[1] - frgb[2] ) / delta;		// between yellow & magenta
	else if( frgb[1] == maximum )
		hsv[0] = 2 + ( frgb[2] - frgb[0] ) / delta;	// between cyan & yellow
	else
		hsv[0] = 4 + ( frgb[0] - frgb[1] ) / delta;	// between magenta & cyan

	hsv[0] *= 60;				// degrees
	if( hsv[0] < 0 )
		hsv[0] += 360;
	return hsv;
}

std::array<uint8_t, 3> CoordinateConverter::imageValToRgb(uint32_t val) {
	std::array<uint8_t, 3> ret;
	ret[0] = val & 0xFF;
	ret[1] = (val >> 8) & 0xFF;
	ret[2] = (val >> 16) & 0xFF;
	return ret;
}

