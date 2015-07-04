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
	initParam.homeTorqueLimit=950;
	initParam.homeHighSpeed=280000;
	initParam.homeLowSpeed=40000;
	initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;

	Aris::RT_CONTROL::ACTUATION cs;

	cs.SetTrajectoryGenerator(tg);
	cs.SysInit(initParam);
	cs.SysInitCommunication();
	cs.SysStart();


	Aris::Core::CONN control_interface;
	control_interface.SetCallBackOnReceivedConnection([](Aris::Core::CONN *pConn,const char *pRemoteIP,int remotePort)
	{
		cout << "control client received:" << endl;
		cout << "    remote ip is:" << pRemoteIP << endl;
		cout << endl;
		return 0;
	});
	control_interface.SetOnReceiveRequest([&cs](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		cout<<"received request"<<endl;
		cs.NRT_PostMsg(msg);
		return MSG();
	});
	control_interface.SetCallBackOnLoseConnection([](Aris::Core::CONN *pConn)
	{
		cout << "control_interface lost" << endl;

		while(true)
		{
			try
			{
				pConn->StartServer("5866");
				break;
			}
			catch(CONN::START_SERVER_ERROR &e)
			{
				cout <<e.what()<<endl<<"will restart in 5s"<<endl;
				usleep(5000);
			}
		}

		return 0;
	});

	control_interface.StartServer("5866");

	Aris::Core::RunMsgLoop();









	return 0;
}
