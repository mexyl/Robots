﻿#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <Aris_Socket.h>
#include <iostream>

using namespace std;

enum ROBOT_CMD
{
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

enum ROBOT_STATE
{
	UNKNOWN,
	UNKNOWN1_HOMED2,
	UNKNOWN1_STARTED2,
	HOMED1_UNKNOWN2,
	HOMED1_STARTED2,
	STARTED1_UNKNOWN2,
	STARTED1_HOMED2,
	STARTED,

	ROBOT_STATE_COUNT
};

extern const int ROBOT_STATE_MACHINE[ROBOT_CMD_COUNT][ROBOT_STATE_COUNT];

extern Aris::Core::CONN control_interface;
extern Aris::Core::CONN visual_interface;
extern Aris::Core::CONN data_interface;



inline void init_interface()
{
	control_interface.SetCallBackOnReceivedConnection([](Aris::Core::CONN *pConn,const char *pRemoteIP,int remotePort)
	{
		cout << "control client received:" << endl;
		cout << "    remote ip is:" << pRemoteIP << endl;
		cout << endl;
		return 0;
	});
	control_interface.SetCallBackOnReceivedData([](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		switch (msg.GetMsgID())
		{
		case 0:break;
		}
		return 0;
	});

	control_interface.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
	{
		cout << "control_interface lost" << endl;
		control_interface.StartServer("5866");
		return 0;
	});
	visual_interface.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
	{
		cout << "visual_interface lost" << endl;
		return 0;
	});
	data_interface.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
	{
		cout << "data_interface lost" << endl;
		return 0;
	});
	
	control_interface.StartServer("5866");
	visual_interface.StartServer("5867");
	data_interface.StartServer("5868");
};


inline void close_interface()
{
	control_interface.Close();
	visual_interface.Close();
	data_interface.Close();
};








#endif