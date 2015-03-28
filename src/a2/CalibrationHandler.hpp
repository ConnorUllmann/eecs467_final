#ifndef CALIBRATION_HANDLER_HPP
#define CALIBRATION_HANDLER_HPP

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <algorithm> 
#include "imagesource/image_u32.h"
#include "Matrix.hpp"
#include "CalibrationInfo.hpp"
#include "BlobDetector.hpp"
#include "math/point.hpp"

using namespace eecs467;

class CalibrationHandler {
private:
	enum CALIBRATION_STATE { IDLE, MASKING1, MASKING2, 
		HSV1, HSV2, HSV3, BOARD1, BOARD2, BOARD3, 
		BOARD4, BOARD5, BOARD6, 
		COORDTRANSFORM };
	CALIBRATION_STATE _state;
	// mask corners are bottom left and top right of screen
	// however it is actually (low, high) and (high, low) of image coordinates
	CalibrationInfo _calibrationInfo;

	int _imageHeight, _imageWidth; // in pixels

	// for getting board transform
	Matrix<float> _imageToGlobal;
	float _armToRightLength; // meters (a0)
	float _armToLeftLength; // meters (a1)

	// std::vector<Point<int>> _imageClicks;
	Point<int> _leftCornerClick;
	std::vector<Point<float>> _armLocations;
	bool _validImageToGlobal;

	pthread_mutex_t _calibMutex;

	CalibrationHandler();
	static CalibrationHandler* _instance;


public:
	static CalibrationHandler* instance();
	/**
	 * @brief call this every time a mouse event triggers
	 * @details pass in the "ground" values of the mouse
	 * 
	 * @param x ground x value
	 * @param y ground y value
	 * @param im ptr to image
	 */
	void handleMouseEvent(double x, double y, image_u32_t* im);

	/**
	 * @brief returns true if nothing is being calibrated
	 */
	bool isIdle();

	/**
	 * @brief sets the internal state to calibrate mask
	 * @details make sure object is idle before calling this
	 */
	void calibrateMask();
	
	/**
	 * @brief sets the internal state to calibrate hsv
	 * @details make sure object is idle before calling this
	 */
	void calibrateHsv();

	/**
	 * @brief saves image height and image width
	 * set mask to true if you want to set the mask
	 * to the whole image
	 */
	void calibrateImageSize(int imageHeight, int imageWidth, bool mask);

	/**
	 * @brief call this only after calibrateHsv has been properly set
	 */
	void calibrateBoardTransform();

	void coordTransform();

	bool boardTransformValid();

	const Matrix<float>& getImageToBoardTransform();

	CalibrationInfo getCalibration();

	int imageWidth();

	int imageHeight();
	
	float armToRightLength();
	
	float armToLeftLength();

	/**
	 * @brief gets the message to display about the state
	 */
	std::string getMessage();

	/**
	 * @brief clips an image based on the corners
	 * @details zeros out the borders of an image
	 * 
	 * @param im image to clip 
	 * @param maskCorners corners that are gotten from getMask()
	 */
	void clipImage(image_u32_t* im);

private:
	static bool createAfflineTransform(
		const std::vector<Point<int>>& imageCoords,
		const std::vector<Point<float>>& armCoords,
		Matrix<float>& afflineTransform);

	static Point<int> getBlobClosestToClick(Point<int> click,
		std::vector<Point<int>> blobs);

	// counter clockwise with respect too screen coordinates
	// clockwise with respect to (x, y) pixel indices
	static void getBlobCounterClockwise(
		std::vector<Point<int>>& blobs, 
		Point<int> start);

	static Point<int> getCentroid(const std::vector<Point<int>>& pts);

	bool putCalibrationToFile();
	
	bool putMaskCalibToFile();

	bool readCalibrationDataFromFile();
	
	bool readMaskDataFromFile();
	

	bool putTransformToFile();

	bool readTransformFromFile();
};

#endif /* CALIBRATION_HANDLER_HPP */
