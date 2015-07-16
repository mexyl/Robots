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
	control_interface.SetCallBackOnReceivedData([&cs](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		cout<<"received data"<<endl;

		ROBOT_CMD cmd;

		msg.PasteAt(&cmd.id,sizeof(int),0);

		msg.PasteAt(&cmd.param[0].toUInt,sizeof(unsigned),4);
		msg.PasteAt(&cmd.param[1].toChar,8,8);
		msg.PasteAt(&cmd.param[2].toChar,8,16);
		msg.PasteAt(&cmd.param[3].toDouble,sizeof(double),24);
		msg.PasteAt(&cmd.param[4].toDouble,sizeof(double),32);
		msg.PasteAt(&cmd.param[5].toDouble,sizeof(double),40);
		msg.PasteAt(&cmd.param[6].toDouble,sizeof(double),48);
		msg.PasteAt(&cmd.param[9].toUInt,sizeof(unsigned),56);

		cout<<"cmd:   "<<cmd.id<<endl;
		cout<<"    total count:   "<<cmd.param[0].toUInt<<endl;
		cout<<"    walk direction:"<<cmd.param[1].toChar<<endl;
		cout<<"    up direction:  "<<cmd.param[2].toChar<<endl;
		cout<<"    step d:        "<<cmd.param[3].toDouble<<endl;
		cout<<"    step h:        "<<cmd.param[4].toDouble<<endl;
		cout<<"    step alpha:    "<<cmd.param[5].toDouble<<endl;
		cout<<"    step beta:     "<<cmd.param[6].toDouble<<endl;
		cout<<"    step number:   "<<cmd.param[9].toUInt<<endl;


		if((cmd.id>=0)&&(cmd.id<=6))
			msg.SetMsgID(EXECUTE_CMD);





		cs.NRT_PostMsg(msg);
		return 0;
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
				usleep(5000000);
			}
		}

		return 0;
	});

	while(true)
	{
		try
		{
			control_interface.StartServer("5866");
			break;
		}
		catch(CONN::START_SERVER_ERROR &e)
		{
			cout <<e.what()<<endl<<"will restart in 5s"<<endl;
			usleep(5000000);
		}
	}

	Aris::Core::RunMsgLoop();









	return 0;
}
