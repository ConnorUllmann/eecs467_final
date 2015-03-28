#ifndef LCM_HANDLER_HPP
#define LCM_HANDLER_HPP

//lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"
#include "lcmtypes/ttt_turn_t.hpp"

class LcmHandler {
private:
	lcm::LCM _lcm;
	pthread_t _lcmThreadPid;

	std::string opponentColor;

	pthread_mutex_t _statusMutex;
	LcmHandler();
	static LcmHandler* _instance;

public:
	static LcmHandler* instance();

	void setOpponentColor(std::string color);

	void handleStatusMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const dynamixel_status_list_t* msg);

	void launchThreads();

	void handleTurnMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan, 
		const ttt_turn_t* msg);


	lcm::LCM* getLcm();

private:
	static void* lcmHandle(void* args);
};

#endif /* LCM_HANDLER_HPP */
