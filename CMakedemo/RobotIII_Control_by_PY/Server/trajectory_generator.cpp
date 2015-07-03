#include "trajectory_generator.h"

Robots::ROBOT_IV robot;


ROBOT_STATE_ID ROBOT_STATE_MACHINE::stateMachine[ROBOT_STATE_COUNT][ROBOT_CMD_COUNT]={INVALID};

ROBOT_STATE_MACHINE::ROBOT_STATE_MACHINE()
{
	ROBOT_STATE_MACHINE::stateMachine[DISABLED][ENABLE]=ENABLED;

	ROBOT_STATE_MACHINE::stateMachine[ENABLED][DISABLE]=DISABLED;
	ROBOT_STATE_MACHINE::stateMachine[ENABLED][HOME_1]=HOMED1_ENABLED2;
	ROBOT_STATE_MACHINE::stateMachine[ENABLED][HOME_2]=ENABLED1_HOMED2;

	ROBOT_STATE_MACHINE::stateMachine[ENABLED1_HOMED2][HOME2START_2]=ENABLED1_STARTED2;

	ROBOT_STATE_MACHINE::stateMachine[ENABLED1_STARTED2][HOME_1]=HOMED1_STARTED2;

	ROBOT_STATE_MACHINE::stateMachine[HOMED1_ENABLED2][HOME2START_1]=STARTED1_ENABLED2;

	ROBOT_STATE_MACHINE::stateMachine[HOMED1_STARTED2][HOME2START_1]=STARTED;

	ROBOT_STATE_MACHINE::stateMachine[STARTED1_ENABLED2][HOME_2]=STARTED1_HOMED2;

	ROBOT_STATE_MACHINE::stateMachine[STARTED1_HOMED2][HOME2START_2]=STARTED;

}

const int motorNum = 1;

const int MapAbsToPhy[18]=
{
			10,	11,	9,
			12,	14,	13,
			17,	15,	16,
			6,	8,	7,
			3,	5,	4,
			0,	2,	1
};
const int MapPhyToAbs[18]=
{
			15,	17,	16,
			12,	14,	13,
			9,	11,	10,
			2,	0,	1,
			3,	5,	4,
			7,	8,	6
};

int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
			-15849882,	 -16354509,	 -16354509,
			-15849882,	 -16354509,	 -16354509,
			-15849882,	 -16354509,	 -16354509,
			-16354509,	 -15849882,	 -16354509,
			-15849882,	 -16354509,	 -16354509,
			-16354509,	 -16354509,  -15849882
};

int enable(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
	static Aris::RT_CONTROL::CMachineData lastCmdData;

	bool isAllRunning=true;

	for (unsigned i=0;i<motorNum;++i)
	{
		if(data.motorsStates[i]==Aris::RT_CONTROL::EMSTAT_RUNNING)
		{
			data.motorsCommands[i]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[i]=lastCmdData.commandData[i];
		}
		else if(data.motorsStates[i]==Aris::RT_CONTROL::EMSTAT_ENABLED)
		{
			data.motorsCommands[i]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[i]=data.feedbackData[i];
			lastCmdData.commandData[i]=data.feedbackData[i];
			isAllRunning=false;
		}
		else
		{
			data.motorsCommands[i]=Aris::RT_CONTROL::EMCMD_ENABLE;
			isAllRunning=false;
		}
	}

	if(isAllRunning)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
int disable(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
	bool isAllDisabled=true;
	for (unsigned i=0;i<motorNum;++i)
	{
		if(data.motorsStates[i]!=Aris::RT_CONTROL::EMSTAT_STOPPED)
		{
			data.motorsCommands[i]=Aris::RT_CONTROL::EMCMD_STOP;
			isAllDisabled=false;
		}
	}

	if(isAllDisabled)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
int home1(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
	static unsigned homeMotors[9]={0,1,2,6,7,8,12,13,14};

	bool isAllHomed=true;

	for(unsigned i=0;i<motorNum;++i)
	{
		if(data.isMotorHomed[homeMotors[i]])
		{
			data.motorsCommands[homeMotors[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[homeMotors[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[homeMotors[i]];
		}
		else
		{
			data.motorsCommands[homeMotors[i]]=Aris::RT_CONTROL::EMCMD_GOHOME;
			data.commandData[homeMotors[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[homeMotors[i]];
			isAllHomed=false;
		}
	}

	if(isAllHomed)
	{
		return 0;
	}
	else
	{
		return -1;
	}
};
int mv_forward(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
const int totalCount=1000;



}


int execute_cmd(unsigned count,const ROBOT_CMD &cmd,Aris::RT_CONTROL::CMachineData &data)
{
	int ret;

	if(count == 0)
	{
		rt_printf("doing cmd with param:%d\n",cmd.param[0].toInt);
	}

	switch (cmd.id)
	{
	case ENABLE:
		ret = enable(count,data);
		break;
	case DISABLE:
		ret = disable(count,data);
		break;
	case HOME_1:
		ret = home1(count,data);
		break;
	}

	return ret;
}




int tg(Aris::RT_CONTROL::CMachineData &data,Aris::Core::RT_MSG &recvMsg,Aris::Core::RT_MSG &sendMsg)
{
	static ROBOT_CMD cmdQueue[10];
	static unsigned currentCmd=0;
	static unsigned cmdNum=0;

	static unsigned count = 0;

	static Aris::RT_CONTROL::CMachineData lastCmdData=data,lastStateData=data;
	static Aris::RT_CONTROL::CMachineData stateData, cmdData;

	stateData=data;
	cmdData=data;

	switch (recvMsg.GetMsgID())
	{
	case EXECUTE_CMD:
		if(cmdNum<9)
		{
			rt_printf("received execute cmd msg\n");
			recvMsg.PasteStruct(cmdQueue[(currentCmd+cmdNum)%10]);
			cmdNum++;
		}
		else
		{
			rt_printf("received execute cmd msg2\n");
		}

		break;
	case MODIFY_PARAM:
		rt_printf("received modify param msg\n");
		break;
	default:
		;
	}


	if(cmdNum>0)
	{
		if(execute_cmd(count,cmdQueue[currentCmd],cmdData)==0)
		{
			count = 0;
			currentCmd= (currentCmd+1)%10;
			cmdNum--;
			rt_printf("cmd finished\n");
		}
		else
		{
			count++;
		}


		if(count%1000==0)
		{
			rt_printf("the cmd is:%d\n",cmdData.motorsCommands[0]);
		}
	}
	else
	{
		cmdData=lastCmdData;
		//
	}


//	static int aaa=0;
//	if(aaa%100==0)
//	{
//		rt_printf("cmd position:%d\n",cmdData.commandData[0].Position);
//		rt_printf("feedback position:%d\n",cmdData.feedbackData[0].Position);
//
//	}
//	aaa++;

	data=cmdData;

	lastStateData=stateData;
	lastCmdData=cmdData;

	return 0;
}
