#ifndef ARM_HPP
#define ARM_HPP

#include <stdint.h>
#include <pthread.h>
#include <array>
#include <vector>
#include <deque>
#include <math.h>
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_command_list_t.hpp"

class Arm {
private:
	dynamixel_status_list_t _list;
	mutable pthread_mutex_t _armMutex;
	Arm();
	static Arm* _instance;
	bool _isMoving;
	std::array<float, 6> _targetPos;
	
	std::deque<dynamixel_command_list_t> _commands;
public:
	static Arm* instance();

	void updateServoAngles(const dynamixel_status_list_t* list);

	bool forwardKinematics(std::array<float, 2>& arr) const;

	static bool inverseKinematics(double to_x, double to_y, double to_z, std::array<float, 6>& arr);

	dynamixel_status_list_t getCurrentStatus() const;

	static dynamixel_status_list_t cmdToStatus(const dynamixel_command_list_t& cmd);

	bool inMotion() const;

	void addHomeCommand();

	void addHomeCommand(float baseAngle);

	void addCommandList(const dynamixel_command_list_t& cmd);

	void addCommandLists(const std::vector<dynamixel_command_list_t>& commands);

	static bool getCommandToPoint(double x, double y, double z, dynamixel_command_list_t& list);

	static void setCommandClawParams(dynamixel_command_list_t& list, 
		float clawAngle, 
		float wristTilt = 0, 
		float wristRotation = 0);

	static std::vector<dynamixel_command_list_t> getCommandByJoints(const dynamixel_command_list_t& list,
		const dynamixel_status_list_t& defaultStatus,
		const std::array<int, 6>& armOrder = std::array<int, 6>{{0, 5, 4, 3, 2, 1}});

	// global coords
	bool addCommandGrabBall(std::array<float, 2> coords);

	// board coords
	bool addCommandDropBall(std::array<int, 2> coords);
};

#endif /* ARM_HPP */
