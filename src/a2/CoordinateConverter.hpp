#ifndef COORDINATE_CONVERTER_HPP
#define COORDINATE_CONVERTER_HPP

#include <stdint.h>
#include <array>
#include <vector>
#include "Matrix.hpp"

// converts between various frames of reference and color

class CoordinateConverter {
public:
	// (r, theta)
	static std::array<float, 2> polarToCartesian(const std::array<float, 2>& arr);

	// angle in radians, (r, theta)
	static std::array<float, 2> cartesianToPolar(const std::array<float, 2>& arr);

	/**
	 * @brief turns screen coordinates to image coordinates
	 * @details NOTE: screen coordinates has the x flipped
	 * top left hand corner of screen is (0, 0) in image
	 * make sure CalibrationHandler is initialized
	 * with image size before this is called
	 * 
	 * @param arr array gotten from mouse even handler
	 * @return (x, y) indices of pixel
	 */
	static std::array<int, 2> screenToImage(const std::array<float, 2>& arr);

	static std::array<float, 2> imageToScreen(const std::array<int, 2>& arr);
	
	static std::array<int, 2> globalToBoard(const std::array<float, 2>& arr);
	
	static std::array<float, 2> boardToGlobal(const std::array<int, 2>& arr);

	static std::array<float, 2> imageToGlobal(const std::array<int, 2>& arr);

	static std::array<float, 3> rgbToHsv(const std::array<uint8_t, 3>& rgb);

	static std::array<uint8_t, 3> imageValToRgb(uint32_t val);
};

#endif /* COORDINATE_CONVERTER_HPP */
