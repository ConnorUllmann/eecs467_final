#include "Arm.hpp"
#include "Constants.hpp"
#include "LcmHandler.hpp"
#include "CoordinateConverter.hpp"
#include "math/angle_functions.hpp"
#include <cmath> 
#include <iostream>

using namespace eecs467;

Arm* Arm::_instance = new Arm;

Arm::Arm() : _isMoving(false) {
	if (pthread_mutex_init(&_armMutex, NULL)) {
		printf("armMutex not initialized\n");
		exit(1);
	}
}

Arm* Arm::instance() {
	return _instance;
}

void Arm::updateServoAngles(const dynamixel_status_list_t* list) {
	
	pthread_mutex_lock(&_armMutex);
	_list = *list;
	//std::cout << "statuses.size() " << _list.statuses.size() << std::endl;
	pthread_mutex_unlock(&_armMutex);
	
	if(_commands.empty()) {
		_isMoving = false;
		return;
	} else {
		_isMoving = true;
	}
	
	LcmHandler::instance()->getLcm()->publish(ARM_CHANNEL_NAME, &_commands.back());

    while (_commands.size() > 1) {
        _commands.pop_front();
    }

	
	// float numFinished = 0;
	// float error = 0;
	// for (int i= 0; i < 6; ++i) {
	// 	float diff = fabs(_list.statuses[i].position_radians -
	// 		_commands.front().commands[i].position_radians);
	// 	error += diff;
		
	// 	//printf("%f\t", diff);
	// 	if((i != 5 && diff < ARM_ERROR_THRESH) || 
	// 	   (i == 5 && diff < HAND_ERROR_THRESH))
	// 		numFinished++;
	// }
	// printf("\nerror: %f\n", error);
	// if (numFinished >= 6) {  //error < ARM_ERROR_THRESH) {
    if (1) {
		pthread_mutex_lock(&_armMutex);
		
		if(!_commands.empty())
		{
			_isMoving = true;
			_commands.pop_front();
		
			if(_commands.empty())
			{
				_isMoving = false;
			}
			else
			{
				_isMoving = true;
				//LcmHandler::instance()->getLcm()->publish(ARM_CHANNEL_NAME, &commands.front());
			}
		} else {
			_isMoving = true;
		}

		pthread_mutex_unlock(&_armMutex);
	}
}


bool Arm::forwardKinematicsPolar(std::array<float, 2>& arr, const dynamixel_status_list_t& list) const {
    // pthread_mutex_lock(&_armMutex);
    if (list.statuses.size() == 0) {
        // pthread_mutex_unlock(&_armMutex);

        return false;
    }

    float length = ARM_LENGTH_1 * sin(list.statuses[1].position_radians) +
                    ARM_LENGTH_2 * sin(angle_sum(list.statuses[1].position_radians, list.statuses[2].position_radians)) +
                    ARM_LENGTH_3 * sin(angle_sum(angle_sum(list.statuses[1].position_radians, list.statuses[2].position_radians), list.statuses[3].position_radians));
    // pthread_mutex_unlock(&_armMutex);

    arr = std::array<float, 2>{{fabsf(length), (float) list.statuses[0].position_radians}};

    return true;
}

bool Arm::forwardKinematicsPolar(std::array<float, 2>& arr) const {
    pthread_mutex_lock(&_armMutex);
    bool ret = forwardKinematicsPolar(arr, _list);
    pthread_mutex_unlock(&_armMutex);
    return ret;
}

bool Arm::forwardKinematics(std::array<float, 2>& arr) const {
	pthread_mutex_lock(&_armMutex);
	if (_list.statuses.size() == 0) {
		pthread_mutex_unlock(&_armMutex);

		return false;
	}

	float length = ARM_LENGTH_1 * sin(_list.statuses[1].position_radians) +
		ARM_LENGTH_2 * sin(_list.statuses[1].position_radians + 
			_list.statuses[2].position_radians) +
		ARM_LENGTH_3 * sin(_list.statuses[1].position_radians + 
			_list.statuses[2].position_radians +
			_list.statuses[3].position_radians);
	float x = length * cos(_list.statuses[0].position_radians);
	float y = length * sin(_list.statuses[0].position_radians);
	pthread_mutex_unlock(&_armMutex);

	arr = std::array<float, 2>{{x, y}};

	return true;
}

bool Arm::inverseKinematicsPolar(double R, double theta, double to_z, std::array<float, 6>& arr)
{
    if (R == 0) {
        std::cout << "Inverse Kinematic Polar R == 0\n"; 
        return false;
    }

    float MSQR = R * R + (ARM_LENGTH_3 + to_z - ARM_LENGTH_0) * (ARM_LENGTH_3 + to_z - ARM_LENGTH_0);

    float alpha = atan2(ARM_LENGTH_3 + to_z - ARM_LENGTH_0, R);

    float beta = acos((-ARM_LENGTH_2 * ARM_LENGTH_2 +  ARM_LENGTH_1 * ARM_LENGTH_1 + MSQR) / (2 * ARM_LENGTH_1 * sqrt(MSQR)));

    float gamma = acos((-MSQR + ARM_LENGTH_1 * ARM_LENGTH_1 + ARM_LENGTH_2 * ARM_LENGTH_2) / (2 * ARM_LENGTH_1 * ARM_LENGTH_2));
    if (isnan(beta) || isnan(gamma)) {
        std::cout << "Inverse Kinematic Polar beta or gamma is nan\n"; 
        return false;
    }

    arr[0] = wrap_to_pi(theta); // arr[0] = wrap_to_pi(M_PI + theta);
    arr[1] = angle_diff(angle_diff(M_PI / 2, alpha), beta);
    arr[2] = angle_diff(M_PI, gamma);
    arr[3] = angle_diff(angle_diff(M_PI, arr[2]), arr[1]);
    arr[4] = 0;//M_PI/2.;
    arr[5] = M_PI/2;//M_PI/2.;

    return true;
}

/*
  The x/y plane is the plane of the surface of the board.
  The z coordinate is the height off of the board.
  Returns a list of the servo angles to set the arm to for it to grab a position.
	 If the function fails, the array will be empty.
  Position (to_x, to_y, to_z) is in the global coordinate frame (meters).
	 It refers to the center of the "palm" surface of the robot.
  Position (base_x, base_y) is in the global coordinate frame (meters).
*/
bool Arm::inverseKinematics(double to_x, double to_y, double to_z, std::array<float, 6>& arr)
{
	double R = sqrt(to_x * to_x + to_y * to_y);
    double theta = atan2(to_y, to_x);

    return inverseKinematicsPolar(R, theta, to_z, arr);
}

dynamixel_status_list_t Arm::getCurrentStatus() const {
	pthread_mutex_lock(&_armMutex);
	dynamixel_status_list_t ret = _list;
	pthread_mutex_unlock(&_armMutex);
	return ret;
}

dynamixel_status_list_t Arm::cmdToStatus(const dynamixel_command_list_t& cmd) {
	dynamixel_status_list_t ret;
	for (int i = 0; i < cmd.len; ++i) {
		dynamixel_status_t status;
		status.position_radians = cmd.commands[i].position_radians;
		ret.statuses.push_back(status);
	}
	ret.len = cmd.len;
	return ret;
}

bool Arm::inMotion() const {
	pthread_mutex_lock(&_armMutex);
	bool ret = _isMoving;
	pthread_mutex_unlock(&_armMutex);
	return ret;
}

void Arm::addHomeCommand() {
	dynamixel_command_list_t cmdList;
	Arm::instance()->getCommandToPoint(0.1, 0, 0.1, cmdList);
	Arm::setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE);
	Arm::instance()->addCommandList(cmdList);

	pthread_mutex_lock(&_armMutex);
	dynamixel_command_list_t list;
	for (int i = 0; i < 6; ++i) {
		dynamixel_command_t cmd;
		cmd.max_torque = ARM_MAX_TORQUE;
		cmd.speed = ARM_SPEED;
		cmd.position_radians = _list.statuses[i].position_radians;
		list.commands.push_back(cmd);
	}
	list.commands[0].position_radians = 0.5;
	list.len = list.commands.size();
	_commands.push_back(list);

	for (int i= 1; i < 6; ++i) {
		list.commands[0].position_radians = 0;
	}
	_commands.push_back(list);
	
	pthread_mutex_unlock(&_armMutex);
}

void Arm::addHomeCommand(float baseAngle) {
	dynamixel_command_list_t list;
	for (int i = 0; i < 6; ++i) {
		dynamixel_command_t cmd;
		cmd.max_torque = ARM_MAX_TORQUE;
		cmd.speed = ARM_SPEED;

		if (i == 0) {
			cmd.position_radians = baseAngle;
		} else if (i == 5) {
			cmd.position_radians = 1.6;
		} else {
			cmd.position_radians = 0;
		}
		list.commands.push_back(cmd);
	}
	list.len = list.commands.size();
	pthread_mutex_lock(&_armMutex);
	_commands.push_back(list);
	_isMoving = true;
	pthread_mutex_unlock(&_armMutex);
}

void Arm::addCommandList(const dynamixel_command_list_t& cmd) {
	pthread_mutex_lock(&_armMutex);
	_commands.push_back(cmd);
	_isMoving = true;
	pthread_mutex_unlock(&_armMutex);
}

void Arm::addCommandLists(const std::vector<dynamixel_command_list_t>& commands) {
	pthread_mutex_lock(&_armMutex);
	for (auto& command : commands) {
		_commands.push_back(command);
	}
	_isMoving = true;
	pthread_mutex_unlock(&_armMutex);
}

bool Arm::getCommandToPolar(double R, double theta, double z, 
        dynamixel_command_list_t& list) {
    list.commands.clear();
    std::array<float, 6> arr;
    if (!inverseKinematicsPolar(R, theta, z, arr)) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        dynamixel_command_t cmd;
        cmd.position_radians = arr[i];
        cmd.max_torque = ARM_MAX_TORQUE;
        cmd.speed = ARM_SPEED;
        list.commands.push_back(cmd);
    }
    list.len = list.commands.size();
    return true;
}

bool Arm::getCommandToPoint(double x, double y, double z, 
	   dynamixel_command_list_t& list) {

    double R = sqrt(x * x + y * y);
    double theta = atan2(y, x);
	return getCommandToPolar(R, theta, z, list) ;
}

void Arm::setCommandClawParams(dynamixel_command_list_t& list, 
		float clawAngle,
		float wristTilt,
		float wristRotation) {
	list.commands[5].position_radians = clawAngle;
	list.commands[4].position_radians = wristRotation;
	list.commands[3].position_radians += wristTilt;
}

std::vector<dynamixel_command_list_t> Arm::getCommandByJoints(const dynamixel_command_list_t& list,
		const dynamixel_status_list_t& defaultStatus,
		const std::array<int, 6>& armOrder) {
	std::vector<dynamixel_command_list_t> ret;


	dynamixel_command_list_t cmdList;
	for (int i = 0; i < 6; ++i) {
		dynamixel_command_t cmd;
		cmd.position_radians = 
			defaultStatus.statuses[i].position_radians;
		cmd.max_torque = ARM_MAX_TORQUE;
		cmd.speed = ARM_SPEED;
		cmdList.commands.push_back(cmd);
	}
	cmdList.len = cmdList.commands.size();

	for (int i = 0; i < 6; ++i) {
		cmdList.commands[armOrder[i]].position_radians = 
			list.commands[armOrder[i]].position_radians;
		ret.push_back(cmdList);
	}

	return ret;
}

bool Arm::addCommandGrabBall(std::array<float, 2> coords) {
	dynamixel_command_list_t cmdList;

	std::array<float, 2> polarCoords = 
		CoordinateConverter::cartesianToPolar(coords);
	polarCoords[1] += BALL_ANGULAR_OFFSET;
	polarCoords[0] -= BALL_RADIAL_OFFSET;
	std::array<float, 2> cartesianCoords = 
		CoordinateConverter::polarToCartesian(polarCoords);

	if (!Arm::getCommandToPoint(cartesianCoords[0], 
		cartesianCoords[1], 0.015, cmdList)) {
		return false;
	} else {
		dynamixel_status_list_t status = Arm::instance()->getCurrentStatus();
		Arm::setCommandClawParams(cmdList, CLAW_OPEN_ANGLE);

		// turn into individual joint motions
		std::vector<dynamixel_command_list_t> cmdListVec =
			Arm::getCommandByJoints(cmdList, status);
		Arm::instance()->addCommandLists(cmdListVec);

		Arm::setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE);
		Arm::instance()->addCommandList(cmdList);

		Arm::instance()->addHomeCommand(cmdList.commands[0].position_radians);
	}
	return true;
}

bool Arm::addCommandDropBall(std::array<int, 2> coords) {
	std::array<float, 2> globalCoords;
	std::array<int, 6> armOrder;
	dynamixel_command_list_t cmdList;
	float wristTilt = 0;
	if (coords[0] == 2) {
		if (coords[1] != 1) {
			wristTilt = -M_PI / 20;
		} else {
			wristTilt = -M_PI / 12;
		}
		globalCoords = CoordinateConverter::boardToGlobal(
			std::array<int, 2>{{coords[0] - 1, coords[1]}});
		globalCoords[1] *= 0.7;
	} else {
		globalCoords = CoordinateConverter::boardToGlobal(coords);
	}
	
	if (coords[0] == 0) {
		armOrder = std::array<int, 6>{{0, 5, 4, 1, 3, 2}};
	} else {
		armOrder = std::array<int, 6>{{0, 5, 4, 3, 2, 1}};
	}


	float baseAngle = 0;
	if (globalCoords[1] < 0) {
		baseAngle = -0.05;
	} else {
		baseAngle = 0.05;
	}

	Arm::instance()->getCommandToPoint(0.1, 0, 0.1, cmdList);
	cmdList.commands[0].position_radians = baseAngle;
	Arm::setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE);
	dynamixel_status_list_t status = Arm::instance()->getCurrentStatus();
	std::vector<dynamixel_command_list_t> cmdListVec =
			Arm::getCommandByJoints(cmdList, status, std::array<int, 6>{{0, 5, 4, 1, 3, 2}});
	status = cmdToStatus(cmdList);
	Arm::instance()->addCommandLists(cmdListVec);


	if (!Arm::instance()->getCommandToPoint(globalCoords[0], globalCoords[1], 0.05, cmdList)) {
		return false;
	} else {
		Arm::setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE, wristTilt);
		
		cmdListVec =
			Arm::getCommandByJoints(cmdList, status, armOrder);
		Arm::instance()->addCommandLists(cmdListVec);

		Arm::setCommandClawParams(cmdList, CLAW_OPEN_ANGLE, wristTilt);
		Arm::instance()->addCommandList(cmdList);
		Arm::instance()->addCommandList(cmdList);


		status = cmdToStatus(cmdList);
		Arm::instance()->getCommandToPoint(globalCoords[0], globalCoords[1], 0.09, cmdList);
		Arm::setCommandClawParams(cmdList, CLAW_OPEN_ANGLE, wristTilt);
		cmdListVec =
			Arm::getCommandByJoints(cmdList, status, armOrder);
		Arm::instance()->addCommandLists(cmdListVec);

		Arm::instance()->addHomeCommand();
	}

	return true;
}


bool Arm::addCommandMoveRotate(double deg) { 
    // add angle go left, sub angle go right

    // std::array<float, 2> arr;

    dynamixel_status_list_t status = Arm::instance()->getCurrentStatus();

// std::cout << "status ";
// for (unsigned int i = 0; i < status.statuses.size(); ++i) {
//     std::cout << status.statuses[i].position_radians << ",";
// }
// std::cout << "\n";


    // if (!forwardKinematicsPolar(arr, status)) {
    //     std::cout << "can't forward kinimatic " << std::endl;
    //     return false;
    // }

    // float newA = arr[1];

    float newA = status.statuses[0].position_radians;

    if (newA + deg > M_PI/2) {
        newA = M_PI/2;
    }
    else if (newA + deg < -M_PI/2) {
        newA = -M_PI/2;
    }
    else {
        newA += deg;
    }

// std::cout << "curR " << arr[0] << ", curA " << arr[1] << ", newA " << newA << std::endl;

    // status.statuses[5].position_radians = status.statuses[0].position_radians;

    dynamixel_command_list_t cmdList;

    // if (!getCommandToPolar(arr[0], newA, 0.02, cmdList)) {
    //     std::cout << "can't rotate to R:" << arr[0] << ", A:" << newA << std::endl;
    //     return false;
    // }
    for (int i = 0; i < 6; ++i) {
        dynamixel_command_t cmd;
        cmd.position_radians = status.statuses[i].position_radians;
        cmd.max_torque = ARM_MAX_TORQUE;
        cmd.speed = ARM_SPEED;
        cmdList.commands.push_back(cmd);
    }

    setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE, 0, newA);

    cmdList.commands[0].position_radians = newA;
    cmdList.commands[1].position_radians = status.statuses[1].position_radians - 0.025;
    cmdList.commands[2].position_radians = status.statuses[2].position_radians - 0.02;
    // cmdList.commands[3].position_radians = status.statuses[3].position_radians;

    cmdList.len = cmdList.commands.size();

// std::cout << "command ";
// for (unsigned int i = 0; i < cmdList.commands.size(); ++i) {
//     std::cout << cmdList.commands[i].position_radians << ",";
// }
// std::cout << "\n";

    Arm::instance()->addCommandList(cmdList);

    return true;
}

bool Arm::addCommandMoveRadiate(double r) {     
    std::array<float, 2> arr;

    dynamixel_status_list_t status = Arm::instance()->getCurrentStatus();

// std::cout << "status ";
// for (unsigned int i = 0; i < status.statuses.size(); ++i) {
//     std::cout << status.statuses[i].position_radians << ",";
// }
// std::cout << "\n";

    if (!forwardKinematicsPolar(arr, status)) {
        std::cout << "can't forward kinimatic " << std::endl;
        return false;
    }

    float newR = arr[0];

    if (arr[0]+r > 0.18) {
        newR = 0.18;
    }
    else if (arr[0]+r < 0.08) {
        newR = 0.08;
    }
    else {
        newR = arr[0]+r;
    }

// std::cout << "curR " << arr[0] << ", newR " << newR << ", curA " << arr[1] << std::endl;

    dynamixel_command_list_t cmdList;

    if (!getCommandToPolar(newR, arr[1], armOffGround, cmdList)) {
        std::cout << "can't radiate to R:" << newR << ", A:" << arr[1] << std::endl;
        return false;
    }

    setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE, 0, arr[1]);

// std::cout << "command ";
// for (unsigned int i = 0; i < cmdList.commands.size(); ++i) {
//     std::cout << cmdList.commands[i].position_radians << ",";
// }
// std::cout << "\n";

    Arm::instance()->addCommandList(cmdList);

    return true;
}

bool Arm::addCommandMoveSwat() {     
    // std::array<float, 2> arr;

    dynamixel_status_list_t status = Arm::instance()->getCurrentStatus();

    dynamixel_command_list_t cmdList;

    for (int i = 0; i < 6; ++i) {
        dynamixel_command_t cmd;
        cmd.position_radians = status.statuses[i].position_radians;
        cmd.max_torque = ARM_MAX_TORQUE;
        cmd.speed = ARM_SPEED*1.5;
        cmdList.commands.push_back(cmd);
    }

    cmdList.len = cmdList.commands.size();

    setCommandClawParams(cmdList, CLAW_OPEN_ANGLE, 0, status.statuses[0].position_radians);

    cmdList.commands[1].position_radians = status.statuses[1].position_radians - 0.025;
    cmdList.commands[2].position_radians = status.statuses[2].position_radians - 0.02;


    while (Arm::instance()->inMotion()) {}
    Arm::instance()->addCommandList(cmdList);

    usleep(1000000/2);

    setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE, 0, status.statuses[0].position_radians);

    Arm::instance()->addCommandList(cmdList);

    return true;
}

bool Arm::addCommandMoveStart() {     
    std::array<float, 2> arr;

    dynamixel_status_list_t status = Arm::instance()->getCurrentStatus();

// std::cout << "status ";
// for (unsigned int i = 0; i < status.statuses.size(); ++i) {
//     std::cout << status.statuses[i].position_radians << ",";
// }
// std::cout << "\n";

    if (!forwardKinematicsPolar(arr, status)) {
        std::cout << "can't forward kinimatic " << std::endl;
        return false;
    }

    dynamixel_command_list_t cmdList;

    if (!getCommandToPolar(borderOffsetX, 0, armOffGround, cmdList)) {
        std::cout << "can't radiate to R:" << 0.1 << ", A:" << 0 << std::endl;
        return false;
    }

    setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE, 0, 0);

// std::cout << "command ";
// for (unsigned int i = 0; i < cmdList.commands.size(); ++i) {
//     std::cout << cmdList.commands[i].position_radians << ",";
// }
// std::cout << "\n";

    Arm::instance()->addCommandList(cmdList);

    return true;
}

bool Arm::addCommandLimp() {     
    
    dynamixel_command_list_t cmdList;

    for (int i = 0; i < 6; ++i) {
        dynamixel_command_t cmd;
        cmd.position_radians = 0;
        cmd.max_torque = 0;
        cmd.speed = 0;
        cmdList.commands.push_back(cmd);
    }

    cmdList.len = cmdList.commands.size();

    Arm::instance()->addCommandList(cmdList);

    return true;
}

bool Arm::addCommandMovePoint(double x, double y) {

    dynamixel_command_list_t cmdList;

    if (!Arm::instance()->getCommandToPoint(x, y, armOffGround, cmdList)) {
        std::cout << "can't move to x:" << x << ", y:" << y << std::endl;
        return false;
    }

    double n = fabs(cmdList.commands[0].position_radians / M_PI * 4);
    double wristAdd = wristAddOn*n*n;

    setCommandClawParams(cmdList, CLAW_CLOSED_ANGLE, wristAdd, cmdList.commands[0].position_radians);

    Arm::instance()->addCommandList(cmdList);
    return true;
}

bool Arm::addCommandMovePoint(std::array<float, 2> coords) {

    return addCommandMovePoint(coords[0], coords[1]);
}
