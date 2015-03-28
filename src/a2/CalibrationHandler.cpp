#include "CalibrationHandler.hpp"
#include "CoordinateConverter.hpp"
#include "ColorRecognizer.hpp"
#include "Constants.hpp"
#include "BlobDetector.hpp"
#include "Arm.hpp"
#include "math/point.hpp"
#include "math/angle_functions.hpp"
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <fstream>

using namespace eecs467;

CalibrationHandler* CalibrationHandler::_instance = new CalibrationHandler;

CalibrationHandler::CalibrationHandler() : _state(IDLE),
	_imageToGlobal(3, 3), _validImageToGlobal(false) {
	if (pthread_mutex_init(&_calibMutex, NULL)) {
		printf("calibration mutex not initialized\n");
		exit(1);
	}
	readCalibrationDataFromFile();
	readTransformFromFile();
	readMaskDataFromFile();
}

CalibrationHandler* CalibrationHandler::instance() {
	return _instance;
}

void CalibrationHandler::handleMouseEvent(double x, double y, image_u32_t* im) {
	std::array<int, 2> imageIndices;
	std::array<uint8_t, 3> rgbVals;

	if (_state != IDLE) {
		imageIndices = CoordinateConverter::screenToImage({{(float)x, (float)y}});
		if (imageIndices[0] < 0) {
			imageIndices[0] = 0;
		}
		if (imageIndices[1] < 0) {
			imageIndices[1] = 0;
		}
		if (imageIndices[0] > im->width - 1) {
			imageIndices[0] = im->width - 1;
		}
		if (imageIndices[1] > im->height - 1) {
			imageIndices[1] = im->height - 1;
		}
		int pixelVal = im->buf[imageIndices[1] * im->stride + imageIndices[0]];
		rgbVals = CoordinateConverter::imageValToRgb(pixelVal);
	}
	pthread_mutex_lock(&_calibMutex);
	switch (_state) {
		case IDLE:
			break;
		case MASKING1:
			_calibrationInfo.maskXRange[0] = imageIndices[0];
			_calibrationInfo.maskYRange[1] = imageIndices[1];
			_state = MASKING2;
			break;
		case MASKING2:
			_calibrationInfo.maskXRange[1] = imageIndices[0];
			_calibrationInfo.maskYRange[0] = imageIndices[1];
			
			putMaskCalibToFile();
			_state = IDLE;
			break;
		case HSV1: {
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgbVals);
			
			_calibrationInfo.redBallHue[0] = hsv[0] - objectHueRedHalfRange;
			_calibrationInfo.redBallHue[1] = hsv[0] + objectHueRedHalfRange;
			_calibrationInfo.sat[0] = hsv[1];			
			_calibrationInfo.val[0] = hsv[2];
				
			printf("rgb: (%d, %d, %d)\n", rgbVals[0], rgbVals[1], rgbVals[2]);

			printf("hsv: (%f, %f, %f)\n", hsv[0], hsv[1], hsv[2]);

			_state = HSV2;
			break; }
		case HSV2: {
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgbVals);

			_calibrationInfo.greenBallHue[0] = hsv[0] - objectHueGreenHalfRange;
			_calibrationInfo.greenBallHue[1] = hsv[0] + objectHueGreenHalfRange;
			_calibrationInfo.sat[0] += hsv[1];			
			_calibrationInfo.val[0] += hsv[2];

			printf("rgb: (%d, %d, %d)\n", rgbVals[0], rgbVals[1], rgbVals[2]);

			printf("hsv: (%f, %f, %f)\n", hsv[0], hsv[1], hsv[2]);

			_state = HSV3;
			break; }
		case HSV3: {
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgbVals);

			_calibrationInfo.blueSquareHue[0] = hsv[0] - objectHueBlueHalfRange;
			_calibrationInfo.blueSquareHue[1] = hsv[0] + objectHueBlueHalfRange;
			_calibrationInfo.sat[0] += hsv[1];			
			_calibrationInfo.val[0] += hsv[2];


			_calibrationInfo.sat[0] /= 3;
			_calibrationInfo.val[0] /= 3;
			_calibrationInfo.sat[0] *= noObjectThresholdSat;
			_calibrationInfo.val[0] *= noObjectThresholdVal;

			_calibrationInfo.sat[1] = 1;
			_calibrationInfo.val[1] = 1;

			printf("rgb: (%d, %d, %d)\n", rgbVals[0], rgbVals[1], rgbVals[2]);
			printf("hsv: (%f, %f, %f)\n", hsv[0], hsv[1], hsv[2]);

			if (!putCalibrationToFile()) {
				printf("Couldn't save calibration to file\n");
			}

			_state = IDLE;
			break; }
		case BOARD1:
		    _armLocations.clear();
			_leftCornerClick = Point<int>{imageIndices[0],
				imageIndices[1]};
			_state = BOARD2;
			break;
		case BOARD2: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_armToLeftLength = sqrt(loc[0] * loc[0] + loc[1] * loc[1]);
			_state = BOARD3;
			break; }
		case BOARD3: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_state = BOARD4;
			break; }
		case BOARD4: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_state = BOARD5;
			break; }
		case BOARD5: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_armToRightLength = sqrt(loc[0] * loc[0] + loc[1] * loc[1]);
			_state = BOARD6;
			break; }
		case BOARD6: {
			// get blobs
			std::vector<BlobDetector::Blob> blobs =
				BlobDetector::findBlobs(im, 
					_calibrationInfo,
					blobMinPixels);

			// only get blue blobs
			std::vector<Point<int>> blueBlobs;
			for (auto& blob : blobs) {
				if (blob.type == BLUESQUARE) {
					blueBlobs.push_back(Point<int>{blob.x, blob.y});
				}
			}
			if (blueBlobs.size() != 4) {
				std::cout << "Not Enough Blue Squares found\n";
				break;
			}

            
			// get blob left of arm
			Point<int> firstBlob = getBlobClosestToClick(_leftCornerClick, blueBlobs);

			// sort the blobs in counterclockwise order
			getBlobCounterClockwise(blueBlobs, firstBlob);
			
			// shifting pts towards center
			//float ratio = (boardDiagonalLength - blueSquareDiagonalLength) / 
			// 	boardDiagonalLength;
			//Point<int> centroid = getCentroid(blueBlobs);
			//for (auto& blob: blueBlobs) {
			//Point<int> delta = blob - centroid;
			//delta.x *= ratio;
			//delta.y *= ratio;
			//    blob = centroid + delta;
		    //}

			if (!createAfflineTransform(blueBlobs, _armLocations,
				_imageToGlobal)) {
				std::cout << "Affline Transform error\n";
				break;
			}

			if (!putTransformToFile()) {
				std::cout << "Unable to save transform to file\n";
			}
			_validImageToGlobal = true;
			_state = IDLE;
			break; }
		case COORDTRANSFORM:
		    pthread_mutex_unlock(&_calibMutex);
			printf("Screen Coords: (%f, %f)\n", x, y);
			printf("Image Coords: (%d, %d)\n", imageIndices[0], imageIndices[1]);
			Matrix<float> imageCoords(3, 1);
			imageCoords(0, 0) = imageIndices[0];
			imageCoords(1, 0) = imageIndices[1];
			imageCoords(2, 0) = 1;
			Matrix<float> globalCoords = _imageToGlobal * imageCoords;
			printf("Global Coords: (%f, %f)\n", globalCoords(0), globalCoords(1));
			std::array<int, 2> boardCoords = CoordinateConverter::globalToBoard(std::array<float, 2>{{globalCoords(0), globalCoords(1)}});
			printf("Board Coords: (%d, %d)\n", boardCoords[0], boardCoords[1]);
			printf("\n");
			
			_state = IDLE;
			pthread_mutex_lock(&_calibMutex);
			break;
	}
	pthread_mutex_unlock(&_calibMutex);
}

bool CalibrationHandler::isIdle() {
	pthread_mutex_lock(&_calibMutex);
	bool ret = _state == IDLE;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

void CalibrationHandler::calibrateMask() {
	pthread_mutex_lock(&_calibMutex);
	_state = MASKING1;
	pthread_mutex_unlock(&_calibMutex);
}

void CalibrationHandler::calibrateHsv() {
	pthread_mutex_lock(&_calibMutex);
	_state = HSV1;
	pthread_mutex_unlock(&_calibMutex);
}

CalibrationInfo CalibrationHandler::getCalibration() {
	pthread_mutex_lock(&_calibMutex);
	CalibrationInfo ret = _calibrationInfo;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

int CalibrationHandler::imageWidth() {
	pthread_mutex_lock(&_calibMutex);
	int ret = _imageWidth;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

int CalibrationHandler::imageHeight() {
	pthread_mutex_lock(&_calibMutex);
	int ret = _imageHeight;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

float CalibrationHandler::armToRightLength() {
    pthread_mutex_lock(&_calibMutex);
	float ret = _armToRightLength;
	pthread_mutex_unlock(&_calibMutex);
    return ret;
}
	
float CalibrationHandler::armToLeftLength() {
    pthread_mutex_lock(&_calibMutex);
	float ret = _armToLeftLength;
	pthread_mutex_unlock(&_calibMutex);
    return ret;
}

void CalibrationHandler::calibrateImageSize(int imageHeight, int imageWidth, bool mask) {
	pthread_mutex_lock(&_calibMutex);
	_imageHeight = imageHeight;
	_imageWidth = imageWidth;
	if (mask) {
		_calibrationInfo.maskXRange[0] = 0;
		_calibrationInfo.maskXRange[1] = imageWidth;
		_calibrationInfo.maskYRange[0] = 0;
		_calibrationInfo.maskYRange[1] = imageHeight;
	}
	pthread_mutex_unlock(&_calibMutex);
}

void CalibrationHandler::calibrateBoardTransform() {
    dynamixel_command_list_t cmdList;
    for (int i = 0; i < 5; ++i) {
        dynamixel_command_t cmd;
        cmd.max_torque = 0;
        cmd.speed = 0;
        cmd.position_radians = 0;
        cmdList.commands.push_back(cmd);
    }
    dynamixel_command_t cmd;
    cmd.max_torque = ARM_MAX_TORQUE;
    cmd.speed = ARM_SPEED;
    cmd.position_radians = 1.9;
    cmdList.commands.push_back(cmd);
    cmdList.len = cmdList.commands.size();
    Arm::instance()->addCommandList(cmdList);

	pthread_mutex_lock(&_calibMutex);
	_state = BOARD1;
	_validImageToGlobal = false;
	pthread_mutex_unlock(&_calibMutex);
}

void CalibrationHandler::coordTransform() {
	pthread_mutex_lock(&_calibMutex);
	_state = COORDTRANSFORM;
	pthread_mutex_unlock(&_calibMutex);
}

bool CalibrationHandler::boardTransformValid() {
	pthread_mutex_lock(&_calibMutex);
	bool ret = _validImageToGlobal;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

const Matrix<float>& CalibrationHandler::getImageToBoardTransform() {
	pthread_mutex_lock(&_calibMutex);
	Matrix<float>& ret = _imageToGlobal;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

std::string CalibrationHandler::getMessage() {
	pthread_mutex_lock(&_calibMutex);
	std::string ret;
	switch(_state) {
		case IDLE:
			ret = ""; break;
		case MASKING1:
			ret = "click bottom left corner"; break;
		case MASKING2:
			ret = "click top right corner"; break;
		case HSV1:
			ret = "click red ball"; break;
		case HSV2:
			ret = "click green ball"; break;
		case HSV3:
			ret = "click blue square"; break;
		case BOARD1:
			ret = "click square to left of arm"; break;
		case BOARD2:
			ret = "move arm to left square, then click"; break;
		case BOARD3:
			ret = "move arm to far left square, then click"; break;
		case BOARD4:
			ret = "move arm to far right square, then click"; break;
		case BOARD5:
			ret = "move arm to right square, then click"; break;
		case BOARD6:
			ret = "make sure image is clear of distractions, then click"; break;
		case COORDTRANSFORM:
			ret = "click a point"; break;
		default:
			ret = ""; break;
	}
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

void CalibrationHandler::clipImage(image_u32_t* im) {
	pthread_mutex_lock(&_calibMutex);
	std::array<int, 2> maskXRange = _calibrationInfo.maskXRange;
	std::array<int, 2> maskYRange = _calibrationInfo.maskYRange;
	pthread_mutex_unlock(&_calibMutex);

	for (int i = 0; i < im->height; ++i) {
		for (int j = 0; j < im->width; ++j) {
			if (i < maskYRange[0] || 
				i > maskYRange[1] || 
				j < maskXRange[0] || 
				j > maskXRange[1]) {
				im->buf[i * im->stride + j] = 0;
			}
		}
	}
}

bool CalibrationHandler::createAfflineTransform(
		const std::vector<Point<int>>& imageCoords,
		const std::vector<Point<float>>& armCoords,
		Matrix<float>& afflineTransform) {

	if (imageCoords.size() != armCoords.size() || imageCoords.size() < 3) {
		return false;
	}

	Matrix<float> imageMatrix(imageCoords.size() * 2, 6);
	Matrix<float> armVector(armCoords.size() * 2, 1);
	imageMatrix.fill(0);
	for (size_t i = 0; i < imageCoords.size(); ++i) {
		int index = 2 * i;
		imageMatrix(index, 0) = imageCoords[i].x;
		imageMatrix(index, 1) = imageCoords[i].y;
		imageMatrix(index, 2) = 1;
		imageMatrix(index + 1, 3) = imageCoords[i].x;
		imageMatrix(index + 1, 4) = imageCoords[i].y;
		imageMatrix(index + 1, 5) = 1;

		armVector(index) = armCoords[i].x;
		armVector(index + 1) = armCoords[i].y;
	}
	
	//imageMatrix.print();
	//armVector.print();

	Matrix<float> x(6, 1);
	Matrix<float>::leastSquares(imageMatrix, armVector, x);

	afflineTransform.fill(0);
	afflineTransform(2, 2) = 1;
	for (int i = 0; i < 6; ++i) {
		afflineTransform(i) = x(i);
	}

	return true;
}

Point<int> CalibrationHandler::getBlobClosestToClick(Point<int> click,
		std::vector<Point<int>> blobs) {
	Point<int> closest = *std::min_element(blobs.begin(),
		blobs.end(),
		[&](const Point<int>& A,
			const Point<int>& B) {
			// just using manhattan distance
			int dist1 = abs(A.x - click.x) + 
				abs(A.y - click.y);

			int dist2 = abs(B.x - click.x) +
				abs(B.y - click.y);
			return dist1 < dist2;
		});
	return closest;
}

void CalibrationHandler::getBlobCounterClockwise(
		std::vector<Point<int>>& blobs, 
		Point<int> start) {
	Point<int> centroid = getCentroid(blobs);

	float startAngle = atan2(start.y - centroid.y, start.x - centroid.x);
	std::sort(blobs.begin(), blobs.end(),
		[&](const Point<int>& A,
			const Point<int>& B) {
			float angle1 = atan2(A.y - centroid.y, A.x - centroid.x);
			float angle2 = atan2(B.y - centroid.y, B.x - centroid.x);

			float diff1 = wrap_to_2pi(angle_diff(angle1, startAngle));
			float diff2 = wrap_to_2pi(angle_diff(angle2, startAngle));

			return diff1 < diff2;

		});
}

Point<int> CalibrationHandler::getCentroid(const std::vector<Point<int>>& pts) {
	Point<int> centroid{0, 0};
	for (auto& pt : pts) {
		centroid.x += pt.x;
		centroid.y += pt.y;
	}
	centroid.x /= pts.size();
	centroid.y /= pts.size();
	return centroid;
}



bool CalibrationHandler::readCalibrationDataFromFile(){
	std::string line;
	std::ifstream inputFile (colorCalibrationFileName.c_str());
	if(!inputFile.is_open()) return false;
	
	inputFile >> _calibrationInfo.redBallHue[0];
	inputFile >> _calibrationInfo.redBallHue[1];
	inputFile >> _calibrationInfo.sat[0];
	inputFile >> _calibrationInfo.sat[1];
	inputFile >> _calibrationInfo.val[0];
	inputFile >> _calibrationInfo.val[1];

	inputFile >> _calibrationInfo.greenBallHue[0];
	inputFile >> _calibrationInfo.greenBallHue[1];
	inputFile >> _calibrationInfo.sat[0];
	inputFile >> _calibrationInfo.sat[1];
	inputFile >> _calibrationInfo.val[0];
	inputFile >> _calibrationInfo.val[1];

	inputFile >> _calibrationInfo.blueSquareHue[0];
	inputFile >> _calibrationInfo.blueSquareHue[1];
	inputFile >> _calibrationInfo.sat[0];
	inputFile >> _calibrationInfo.sat[1];
	inputFile >> _calibrationInfo.val[0];
	inputFile >> _calibrationInfo.val[1];

	return true;

}
bool CalibrationHandler::putCalibrationToFile(){
	std::ofstream outputFile;
	outputFile.open(colorCalibrationFileName.c_str());
	if(!outputFile.is_open()) return false;

	// Red //
	outputFile << _calibrationInfo.redBallHue[0] << " " << _calibrationInfo.redBallHue[1] << " " 
		   << _calibrationInfo.sat[0] << " " << _calibrationInfo.sat[1] << " " 
		   << _calibrationInfo.val[0] << " " << _calibrationInfo.val[1] << "\n";
	// Green //
	outputFile << _calibrationInfo.greenBallHue[0] << " " << _calibrationInfo.greenBallHue[1] << " " 
		   << _calibrationInfo.sat[0] << " " << _calibrationInfo.sat[1] << " " 
		   << _calibrationInfo.val[0] << " " << _calibrationInfo.val[1] << "\n";
	// Blue //
	outputFile << _calibrationInfo.blueSquareHue[0] << " " << _calibrationInfo.blueSquareHue[1] << " " 
		   << _calibrationInfo.sat[0] << " " << _calibrationInfo.sat[1] << " " 
		   << _calibrationInfo.val[0] << " " << _calibrationInfo.val[1] << "\n";

	outputFile.close();
	return true;
}

bool CalibrationHandler::readMaskDataFromFile(){
	std::string line;
	std::ifstream inputFile (maskCalibFileName.c_str());
	if(!inputFile.is_open()) return false;
	
	inputFile >> _calibrationInfo.maskXRange[0];
	inputFile >> _calibrationInfo.maskXRange[1];
	inputFile >> _calibrationInfo.maskYRange[0];
	inputFile >> _calibrationInfo.maskYRange[1];

	return true;
}

bool CalibrationHandler::putMaskCalibToFile(){
	std::ofstream outputFile;
	outputFile.open(maskCalibFileName.c_str());
	if(!outputFile.is_open()) return false;

	// Red //
	outputFile << _calibrationInfo.maskXRange[0] << " " << _calibrationInfo.maskXRange[1] << "\n";
	outputFile << _calibrationInfo.maskYRange[0] << " " << _calibrationInfo.maskYRange[1] << "\n";
	
	outputFile.close();
	return true;
}

bool CalibrationHandler::putTransformToFile() {
	std::ofstream outputFile;
	outputFile.open(transformCalibFileName.c_str());
	if (!outputFile.is_open()) return false;

	for (int i = 0; i < 9; ++i) {
		outputFile << _imageToGlobal(i) <<  "\t";
	}
	outputFile << "\n";
	outputFile << _armToRightLength << "\t";
	outputFile << _armToLeftLength;
	outputFile.close();
	return true;
}

bool CalibrationHandler::readTransformFromFile() {
	std::ifstream inputFile(transformCalibFileName.c_str());
	if(!inputFile.is_open()) return false;

	for (int i = 0; i < 9; ++i) {
		inputFile >> _imageToGlobal(i);
	}
	inputFile >> _armToRightLength;
	inputFile >> _armToLeftLength;
	
	inputFile.close();
	return true;
}
