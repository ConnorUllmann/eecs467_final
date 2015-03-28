#include "a2/Arm.hpp"
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"
#include "lcmtypes/ttt_turn_t.hpp"


int main() {
	lcm::LCM lcm;
        int64_t utime = 0;
        int32_t turn = 0;

	ttt_turn_t message;

	message.utime = utime;
	message.turn = turn;
	while (1) {
		std::cout << "Press Enter to move";
		std::cin.get();
		lcm.publish("GREEN_TURN", &message);
		
		message.turn++;
	}
}



