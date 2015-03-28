#include "ColorRecognizer.hpp"
#include "CoordinateConverter.hpp"
#include "Constants.hpp"
#include "math/angle_functions.hpp"
#include "CoordinateConverter.hpp"
#include "GamePlayer.hpp"
#include <stdio.h>


OBJECT determineObjectHSV(std::array<float, 3> hsv, const CalibrationInfo &c) {
	/*float satThreshold = ((float)(c.redBallHSV[1] + c.greenBallHSV[1] + c.blueSquareHSV[1]) / 3) * noObjectThresholdVal;
	float valThreshold = ((float)(c.redBallHSV[2] + c.greenBallHSV[2] + c.blueSquareHSV[2]) / 3) * noObjectThresholdSat;*/

	if(hsv[1] < c.sat[0] || hsv[1] > c.sat[1])
		return NONE;
	if(hsv[2] < c.val[0] || hsv[2] > c.val[1])
		return NONE;
	if(eecs467::angle_between(c.redBallHue[0], c.redBallHue[1], hsv[0]))
		return REDBALL;
	if(eecs467::angle_between(c.greenBallHue[0], c.greenBallHue[1], hsv[0]))
		return GREENBALL;
	if(eecs467::angle_between(c.blueSquareHue[0], c.blueSquareHue[1], hsv[0]))
		return BLUESQUARE;
	return NONE;
}

void maskWithColors(image_u32_t* im, const CalibrationInfo& c) {
	for (int row = 0; row < im->height; row++) {
		for (int col = 0; col < im->width; col++) {
			if (row < c.maskYRange[0] || row > c.maskYRange[1] ||
				col < c.maskXRange[0] || col > c.maskXRange[1]) {
				im->buf[row * im->stride + col] = 0x00;
				continue;
			}

			uint32_t val = im->buf[row * im->stride + col];
			std::array<uint8_t, 3> rgbVals;
			rgbVals[0] = val & 0xFF;
			rgbVals[1] = (val >> 8) & 0xFF;
			rgbVals[2] = (val >> 16) & 0xFF;

			OBJECT outcome = determineObjectHSV(CoordinateConverter::rgbToHsv(rgbVals), c);

			// printf("outcome: %d\n", outcome);

			switch (outcome) {
				case REDBALL:
					im->buf[row * im->stride + col] = 0xFF0000FF;
					break;
				case GREENBALL:
					im->buf[row * im->stride + col] = 0xFF00FF00;
					break;
				case BLUESQUARE:
					im->buf[row * im->stride + col] = 0xFFFF0000;
					break;
				default:
					im->buf[row * im->stride + col] = 0x00;
					break;
			}
		}
	}
}

void maskWithBoard(image_u32_t* im, const CalibrationInfo& c) {
	for (int row = 0; row < im->height; row++) {
		for (int col = 0; col < im->width; col++) {
			if (row < c.maskYRange[0] || row > c.maskYRange[1] ||
				col < c.maskXRange[0] || col > c.maskXRange[1]) {
				im->buf[row * im->stride + col] = 0x00;
				continue;
			}

			uint32_t val = im->buf[row * im->stride + col];
			std::array<uint8_t, 3> rgbVals;
			rgbVals[0] = val & 0xFF;
			rgbVals[1] = (val >> 8) & 0xFF;
			rgbVals[2] = (val >> 16) & 0xFF;
			
			std::array<int, 2> pix;
			pix[0] = col;
			pix[1] = row;
			auto arr = CoordinateConverter::globalToBoard(CoordinateConverter::imageToGlobal(pix));
            
            if(arr[0] >= 0 && arr[1] >= 0 && arr[0] < 3 && arr[1] < 3)
            {
            	auto p = GamePlayer::instance()->_board.board[arr[0]][arr[1]];
            	switch(p)
            	{
            	case REDBALL:
            		im->buf[row * im->stride + col] = 0xFF000000 + (rgbVals[2] << 16) + (rgbVals[1] << 8) + 0xFF;
            		break;
            	case GREENBALL:
            		im->buf[row * im->stride + col] = 0xFF000000 + (rgbVals[2] << 16) + 0xFF00 + rgbVals[0];
            		break;
            	default:
            		break;
            	}
                //im->buf[row * im->stride + col] = 0xFF000000 + ((0x87 * arr[0] + rgbVals[2]) << 16) + ((0x87 * arr[1] + rgbVals[1]) << 8) + rgbVals[0];
            }
            else
                im->buf[row * im->stride + col] = val - 0xCC000000;
		}
	}
}

