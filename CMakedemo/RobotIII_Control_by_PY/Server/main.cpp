#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>

using namespace Aris::Core;

#include "robot_interface.h"
#include "trajectory_generator.h"




int main()
{
	Aris::RT_CONTROL::CSysInitParameters initParam;

	int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
	{
			-15849882,	 -16354509,	 -16354509,
			-15849882,	 -16354509,	 -16354509,
			-15849882,	 -16354509,	 -16354509,
			-16354509,	 -15849882,	 -16354509,
			-15849882,	 -16354509,	 -16354509,
			-16354509,	 -16354509,  -15849882
	};

	initParam.motorNum=18;
	initParam.homeHighSpeed=280000;
	initParam.homeLowSpeed=40000;
	initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;

	Aris::RT_CONTROL::ACTUATION cs;

	cs.SetTrajectoryGenerator(tg);
	cs.SysInit(initParam);
	cs.SysInitCommunication();
	cs.SysStart();


	while(true)
	{
		usleep(100000);

		MSG msg(0);

		ROBOT_CMD cmd;

		cmd.id=1;
		cmd.paramNum=3;
		cmd.param[0].toInt=0;





		cs.NRT_PostMsg(msg);


	}




	//Aris::Core::RunMsgLoop();
	//int cmd;
	//while(cin>>)









	return 0;
}
