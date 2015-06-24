#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>

#include <Aris_Core.h>
#include <Aris_Message.h>

using namespace Aris::Core;

#include "robot_interface.h"


void printCMD(int cmd)
{
	switch (cmd)
	{
	case HOME_1: std::cout << "Home1"; break;
	case HOME_2: std::cout << "Home2"; break;
	case HOME2START_1: std::cout << "Home2Start1"; break;
	case HOME2START_2: std::cout << "Home2Start2"; break;
	case MV_FORWARD: std::cout << "Mv_Forward"; break;
	case MV_BACKWARD: std::cout << "Mv_Backward"; break;
	case TURN_LEFT: std::cout << "Turn_Left"; break;
	case TURN_RIGHT: std::cout << "Turn_Right"; break;
	}
}


void printState(int state)
{
	switch (state)
	{
	case UNKNOWN: std::cout << "UNKNOWN"; break;
	case STARTED: std::cout << "STARTED"; break;
	case UNKNOWN1_HOMED2: std::cout << "UNKNOWN1_HOMED2"; break;
	case UNKNOWN1_STARTED2: std::cout << "UNKNOWN1_STARTED2"; break;
	case HOMED1_UNKNOWN2: std::cout << "HOMED1_UNKNOWN2"; break;
	case HOMED1_STARTED2: std::cout << "HOMED1_STARTED2"; break;
	case STARTED1_UNKNOWN2: std::cout << "STARTED1_UNKNOWN2"; break;
	case STARTED1_HOMED2: std::cout << "STARTED1_HOMED2"; break;
	}
}


int main()
{
	char RemoteIp[] = "127.0.0.1";

	/*for (unsigned i = 0; i < ROBOT_CMD_COUNT; ++i)
	{
		for (unsigned j = 0; j < ROBOT_STATE_COUNT; ++j)
		{
			if (ROBOT_STATE_MACHINE[i][j] != -1)
			{
				printCMD(i);
				std::cout << "->";
				printState(j);
				std::cout << " : ";
				printState(ROBOT_STATE_MACHINE[i][j]);
				std::cout << std::endl;

			}
		}
	}*/


	init_interface();







	Aris::Core::RunMsgLoop();


	//close_interface();
}
