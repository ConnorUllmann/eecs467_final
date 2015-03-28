#ifndef COLOR_RECOGNIZER_HPP
#define COLOR_RECOGNIZER_HPP

#include <math.h>

#include <array>
#include "CalibrationInfo.hpp"

#include "imagesource/image_u32.h"

enum OBJECT : uint8_t { NONE, REDBALL, GREENBALL, BLUESQUARE };

OBJECT determineObjectRGB(std::array<uint8_t, 3> rgb);

OBJECT determineObjectRGB2(std::array<uint8_t, 3> rgb, const CalibrationInfo& c);

OBJECT determineObjectHSV(std::array<float, 3> hsv, const CalibrationInfo &c);

void maskWithColors(image_u32_t* im, const CalibrationInfo& c);

void maskWithBoard(image_u32_t* im, const CalibrationInfo& c);

#endif /* COLOR_RECOGNIZER_HPP */
