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

enum ROBOT_CMD_ID
{
	ENABLE,
	DISABLE,
	HOME,
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
		char toChar[8];
		unsigned toUInt;
		int toInt;
		float toFload;
		double toDouble;
	};

	PARAM param[10];
};


#endif
