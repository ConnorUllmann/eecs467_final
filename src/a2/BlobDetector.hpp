#ifndef BLOB_DETECTOR_HPP
#define BLOB_DETECTOR_HPP

#include <vector>
#include <iostream>
#include <stdint.h>
#include "imagesource/image_u32.h"
#include "ColorRecognizer.hpp"
#include "Matrix.hpp"


namespace BlobDetector {

struct Blob {
	int x, y;
	int size; // number of pixels
	OBJECT type;
};

// for use in findBlobs
struct BlobCell {
	OBJECT type;
	bool partOfBlob;
};


/**
 * @brief finds blobs in an image
 * @details uses ColorRecognizer to find colors of objects
 * 
 * @param im image to process
 * @param calib calibration info to determine what pixels are what colors
 * @param minPixels, the minimum number of pixels required to be considered a blob
 * @return vector of blobs
 */
std::vector<Blob> findBlobs(image_u32_t* im, const CalibrationInfo& calib, size_t minPixels);

// for use in findBlobs
std::vector<Blob> findBlobsFromMatrix(Matrix<BlobCell>& mat, const CalibrationInfo& calib, size_t minPixels);


}


#endif /* BLOB_DETECTOR_HPP */
