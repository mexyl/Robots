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


extern int HEXBOT_HOME_OFFSETS_RESOLVER[18];

int main()
{
	Aris::RT_CONTROL::CSysInitParameters initParam;

	initParam.motorNum=18;
	initParam.homeMode=-1;
	initParam.homeTorqueLimit=200;
	initParam.homeHighSpeed=280000;
	initParam.homeLowSpeed=40000;
	initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;

	Aris::RT_CONTROL::ACTUATION cs;

	cs.SetTrajectoryGenerator(tg);
	cs.SysInit(initParam);
	cs.SysInitCommunication();
	cs.SysStart();


	cout<<"now send cmd"<<endl;


	MSG msg(EXECUTE_CMD);


	ROBOT_CMD cmd;

	cmd.id=DISABLE;
	msg.CopyStruct(cmd);
	cs.NRT_PostMsg(msg);

	usleep(5000000);

	cmd.id=ENABLE;
	msg.CopyStruct(cmd);
	cs.NRT_PostMsg(msg);

	cmd.id=HOME_1;
	msg.CopyStruct(cmd);
	cs.NRT_PostMsg(msg);

	usleep(10000000);

	cmd.id=DISABLE;
	msg.CopyStruct(cmd);
	cs.NRT_PostMsg(msg);


	int i=0;
//	while(i<5)
//	{
//		usleep(500000);
//
//		ROBOT_CMD cmd;
//
//			cmd.id=ENABLE;
//			cmd.paramNum=3;
//			cmd.param[0].toInt=i;
//			i++;
//			cmd.param[1].toInt=2;
//			cmd.param[2].toInt=3;
//
//			msg.CopyStruct(cmd);
//			cs.NRT_PostMsg(msg);
//	}
//
//	usleep(10000000);
//
//	i=0;
//		while(i<25)
//		{
//			usleep(100000);
//
//			ROBOT_CMD cmd;
//
//				cmd.id=ENABLE;
//				cmd.paramNum=3;
//				cmd.param[0].toInt=i;
//				i++;
//				cmd.param[1].toInt=2;
//				cmd.param[2].toInt=3;
//
//				msg.CopyStruct(cmd);
//				cs.NRT_PostMsg(msg);
//		}




	Aris::Core::RunMsgLoop();
	//int cmd;
	//while(cin>>)









	return 0;
}
