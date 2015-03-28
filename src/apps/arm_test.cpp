#include "a2/Arm.hpp"
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"


int main() {
	lcm::LCM lcm;

	float x, y;
	while (1) {
		std::cout << "x: ";
		std::cin >> x;
		std::cout << "y: ";
		std::cin >> y; 


		// Arm::instance()->moveArm(x, y, 0.05);
		// std::array<float, 6> arr;
		// if (!Arm::instance()->inverseKinematics(x, y, 0.02, arr)) {
		// 	std::cout << "Impossible!\n";
		// 	continue;
		// }

		// dynamixel_command_list_t list;
		// for (int i = 0; i < 6; ++i) {
		// 	dynamixel_command_t cmd;
		// 	cmd.position_radians = arr[i];
		// 	cmd.speed = 0.2;
		// 	cmd.max_torque = 0.4;
		// 	list.commands.push_back(cmd);
		// }
		// list.len = list.commands.size();
		// lcm.publish("ARM_COMMAND", &list);
	}
}



