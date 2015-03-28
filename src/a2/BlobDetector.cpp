#include "BlobDetector.hpp"
#include "Matrix.hpp"
#include "CoordinateConverter.hpp"

using namespace BlobDetector;

///////////////////////////
// "PRIVATE" FUNCTIONS
///////////////////////////
/**
 * @brief takes an image and turns it into a matrix of BlobCells
 * that can be processed later
 */
Matrix<BlobCell> imageToMatrix(image_u32_t* im, const CalibrationInfo& calib);
/**
 * @brief return vector of all (x, y) coordinates related to a blob
 * @details will modify mat
 * 
 * @param mat Matrix containing info for blobs
 * @param (x,y) location to start expanding the blob. it will determine blob type
 */
std::vector<std::array<int, 2>> findAndMarkBlob(Matrix<BlobCell>& mat, int x, int y);
/**
 * @brief finds centroid of vector (x, y) points
 */
std::array<int, 2> findCentroid(std::vector<std::array<int, 2>>& points);


std::vector<Blob> BlobDetector::findBlobs(image_u32_t* im, const CalibrationInfo& calib, size_t minPixels) {
	// turn image into matrix of BlobCells
	Matrix<BlobCell> mat = imageToMatrix(im, calib);

	return findBlobsFromMatrix(mat, calib, minPixels);
}

std::vector<Blob> BlobDetector::findBlobsFromMatrix(Matrix<BlobCell>& mat, const CalibrationInfo& calib, size_t minPixels) {
	std::vector<Blob> ret;
	for (int row = calib.maskYRange[0]; row < calib.maskYRange[1]; ++row) {
		for (int col = calib.maskXRange[0]; col < calib.maskXRange[1]; ++col) {
			BlobCell cell = mat(row, col);
			if (cell.type != NONE && !cell.partOfBlob) {
				std::vector<std::array<int, 2>> currBlob = findAndMarkBlob(mat, col, row);
				if (currBlob.size() < minPixels) {
					continue;
				}
				std::array<int, 2> center = findCentroid(currBlob);
				ret.push_back({center[0], center[1], (int)currBlob.size(), cell.type});
			}
		}
	}
	return ret;
}

Matrix<BlobCell> imageToMatrix(image_u32_t* im, const CalibrationInfo& calib) {
	Matrix<BlobCell> ret(im->height, im->width, BlobCell());

	for (int row = calib.maskYRange[0]; row < calib.maskYRange[1]; ++row) {
		for (int col = calib.maskXRange[0]; col < calib.maskXRange[1]; ++col) {
			uint32_t val = im->buf[row * im->stride + col];
			std::array<uint8_t, 3> rgb = CoordinateConverter::imageValToRgb(val);
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgb);
			OBJECT type = determineObjectHSV(hsv, calib);
			ret(row, col) = {type, false};
		}
	}

	return ret;
}

std::vector<std::array<int, 2>> findAndMarkBlob(Matrix<BlobCell>& mat, int x, int y) {
	std::vector<std::array<int, 2>> ret;
	std::vector<std::array<int, 2>> toBeProcessed;
	OBJECT blobType = mat(y, x).type;
	mat(y, x).partOfBlob = true;
	toBeProcessed.push_back({{x, y}});

	while (!toBeProcessed.empty()) {
		std::array<int, 2> curr = toBeProcessed.back();
		toBeProcessed.pop_back();
		ret.push_back(curr);

		for (int row = -1; row <= 1; ++row) {
			for (int col = -1; col <= 1; ++col) {
				if (mat.validIndex(curr[1] + row, curr[0] + col)) {
					BlobCell& currCell = mat(curr[1] + row, curr[0] + col);
					if (currCell.type == blobType && currCell.partOfBlob == false) {
						currCell.partOfBlob = true;
						toBeProcessed.push_back({{curr[0] + col, curr[1] + row}});
					}
				}
			}
		}
	}
	return ret;
}

std::array<int, 2> findCentroid(std::vector<std::array<int, 2>>& points) {
	std::array<int, 2> ret{{0, 0}};
	for (const auto& point : points) {
		ret[0] += point[0];
		ret[1] += point[1];
	}
	ret[0] /= points.size();
	ret[1] /= points.size();

	return ret;
}
