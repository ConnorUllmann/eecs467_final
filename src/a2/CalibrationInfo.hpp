#ifndef CALIBRATION_INFO_HPP
#define CALIBRATION_INFO_HPP


struct CalibrationInfo {
	std::array<int, 2> maskXRange; // (low, high) x range
	std::array<int, 2> maskYRange; // (low, high) y range

	std::array<float, 2> redBallHue;
	std::array<float, 2> greenBallHue;
	std::array<float, 2> blueSquareHue;
	std::array<float, 2> sat;
	std::array<float, 2> val;

	CalibrationInfo() :
		maskXRange{{0, 100}},
		maskYRange{{0, 100}},
		redBallHue{{0, 0}},
		greenBallHue{{0, 0}},
		blueSquareHue{{0, 0}},
		sat{{0, 0}},
		val{{0, 0}}{ }

};

#endif /* CALIBRATION_INFO_HPP */
