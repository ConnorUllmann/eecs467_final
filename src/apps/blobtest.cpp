#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

#include <iostream>

#include "VxHandler.hpp"
#include "a2/ColorRecognizer.hpp"
#include "a2/CoordinateConverter.hpp"
#include "a2/Arm.hpp"
#include "a2/BlobDetector.hpp"
#include "math/angle_functions.hpp"

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

//lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"

#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"


using namespace std;
using namespace BlobDetector;
using namespace eecs467;


void printMatrix(Matrix<BlobCell>& mat) {
	for (int row = 0; row < mat.rows(); ++row) {
		for (int col = 0; col < mat.cols(); ++col) {
			std::cout << mat(row, col).type << "\t";
		}
		std::cout << "\n";
	}
}


void printCommandList(dynamixel_command_list_t list) {
	for (auto& cmd : list.commands) {
		printf("%f\t", cmd.position_radians);
	}
	printf("\n");
}

int main(){
	dynamixel_command_list_t cmdList;
	Arm::getCommandToPoint(0.1, 0, 0, cmdList);
	printf("target: ");
	printCommandList(cmdList);
	dynamixel_status_list_t statusList;
	std::array<float, 6> posRadians{{0, 0, 0, -0.1, -0.1, 1.58}};
	for (int i = 0; i < 6; ++i){
		dynamixel_status_t status;
		status.position_radians = posRadians[i];
		statusList.statuses.push_back(status);
	}
	std::vector<dynamixel_command_list_t> cmds = Arm::getCommandByJoints(
		cmdList, statusList);
	for (auto& cmd : cmds) {
		printCommandList(cmd);
	}



	// std::cout << angle_between(0, 100, 10) << "\n";
	// std::cout << angle_between(340, 20, 10) << "\n";

	// std::cout << angle_between(0, 100, 110) << "\n";
	// std::cout << angle_between(340, 20, 320) << "\n";
	// std::cout << angle_between(340, 20, 40) << "\n";

	// std::vector<BlobDetector::Blob> blobs{
	// 	{1, 1},
	// 	{11, 10},
	// 	{1, 12},
	// 	{9, 1}
	// };
	// CalibrationHandler::getBlobCounterClockwise(blobs, blobs.front());

	// for (auto& blob: blobs) {
	// 	printf("%d, %d\n", blob.x, blob.y);
	// }

	// std::array<float, 6> angles;
	// if (!Arm::inverseKinematics(0.05, 0.15, 0, angles)) {
	// 	std::cout << "IMPOSSIBLE!\n";
	// }

	// for (auto angle: angles) {
	// 	std::cout << (float)(angle * 180) / M_PI << "\t";
	// }
	// std::cout << "\n";

	// dynamixel_status_list_t list;

	// for (int i = 0; i < 6; ++i) {
	// 	dynamixel_status_t status;
	// 	status.position_radians = angles[i];
	// 	list.statuses.push_back(status);
	// }

	// Arm::instance()->updateServoAngles(&list);


	// std::array<float, 2> arr;
	// Arm::instance()->forwardKinematics(arr);
	// printf("%f, %f\n", arr[0], arr[1]);
}
