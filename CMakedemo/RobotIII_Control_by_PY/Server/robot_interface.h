#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <Aris_Socket.h>
#include <iostream>

using namespace std;

enum CLIENT_CMD_ID
{
	EXECUTE_CMD,
	MODIFY_PARAM,

	CLIENT_CMD_COUNT
};

enum ROBOT_STATE_ID
{
	INVALID,
	DISABLED,
	ENABLED,
	ENABLED1_HOMED2,
	ENABLED1_STARTED2,
	HOMED1_ENABLED2,
	HOMED1_STARTED2,
	STARTED1_ENABLED2,
	STARTED1_HOMED2,
	STARTED,

	ROBOT_STATE_COUNT
};

enum ROBOT_CMD_ID
{
	ENABLE,
	DISABLE,
	HOME_1,
	HOME_2,
	HOME2START_1,
	HOME2START_2,
	MV_FORWARD,
	MV_BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,

	ROBOT_CMD_COUNT
};

struct ROBOT_CMD
{
	ROBOT_CMD_ID id;
	int paramNum;

	union PARAM
	{
		int toInt;
		float toFload;
		double toDouble;
	};

	PARAM param[10];
};


#endif
