#include "trajectory_generator.h"
#include <cstring>
#include <Aris_Plan.h>
#include <Robot_Gait.h>

using namespace std;


Robots::ROBOT_IV robot;

int HEXBOT_HOME_OFFSETS_RESOLVER[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const double meter2count = 1/0.005*1.5*65536;
const int motorNum=18;
/*
int enable(int count , Aris::RT_CONTROL::CMachineData &data)
{
	static Aris::RT_CONTROL::CMachineData lastCmdData;

	bool isAllRunning=true;

	for (int i=0;i<motorNum;++i)
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
int disable(int count , Aris::RT_CONTROL::CMachineData &data)
{
	bool isAllDisabled=true;
	for (int i=0;i<motorNum;++i)
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

int home1(int count , Aris::RT_CONTROL::CMachineData &data)
{
	static int homeMotors[9]={0,1,2,6,7,8,12,13,14};

	bool isAllHomed=true;

	for(int i=0;i<9;++i)
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
};*/
int home(int count,int motorNum, const int *motorID, Aris::RT_CONTROL::CMachineData &data)
{
	bool isAllHomed=true;

	for(int i=0;i<9;++i)
	{
		if(data.isMotorHomed[motorID[i]])
		{
			data.motorsCommands[motorID[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[motorID[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[motorID[i]];
		}
		else
		{
			data.motorsCommands[motorID[i]]=Aris::RT_CONTROL::EMCMD_GOHOME;
			data.commandData[motorID[i]].Position=-HEXBOT_HOME_OFFSETS_RESOLVER[motorID[i]];
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
int enable(int count,int motorNum, const int *motorID, Aris::RT_CONTROL::CMachineData &data)
{
	static Aris::RT_CONTROL::CMachineData lastCmdData;

	bool isAllRunning=true;

	for (int i=0;i<motorNum;++i)
	{
		if(data.motorsStates[motorID[i]]==Aris::RT_CONTROL::EMSTAT_RUNNING)
		{
			data.motorsCommands[motorID[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[motorID[i]]=lastCmdData.commandData[motorID[i]];
		}
		else if(data.motorsStates[motorID[i]]==Aris::RT_CONTROL::EMSTAT_ENABLED)
		{
			data.motorsCommands[motorID[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[motorID[i]]=data.feedbackData[motorID[i]];
			lastCmdData.commandData[motorID[i]]=data.feedbackData[motorID[i]];
			isAllRunning=false;
		}
		else
		{
			data.motorsCommands[motorID[i]]=Aris::RT_CONTROL::EMCMD_ENABLE;
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
};

int disable(int count,int motorNum, const int *motorID, Aris::RT_CONTROL::CMachineData &data)
{
	bool isAllDisabled=true;
	for (int i=0;i<motorNum;++i)
	{
		if(data.motorsStates[motorID[i]]!=Aris::RT_CONTROL::EMSTAT_STOPPED)
		{
			data.motorsCommands[motorID[i]]=Aris::RT_CONTROL::EMCMD_STOP;
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



int execute_cmd(int count,const char * cmd, Aris::RT_CONTROL::CMachineData &data)
{
	int ret;

	int id = *reinterpret_cast<const int *>(cmd);



	switch (id)
	{
	case ENABLE:
	{
		int num=*reinterpret_cast<const int *>(cmd+sizeof(int));
		ret = enable(count,num, reinterpret_cast<const int *>(cmd+2*sizeof(int)) , data);
		break;
	}
	case DISABLE:
	{
		int num=*reinterpret_cast<const int *>(cmd+sizeof(int));
		ret = disable(count,num, reinterpret_cast<const int *>(cmd+2*sizeof(int)) ,data);
		break;
	}
	case HOME:
	{
		int num=*reinterpret_cast<const int *>(cmd+sizeof(int));
		ret = home(count,num, reinterpret_cast<const int *>(cmd+2*sizeof(int)) ,data);
		break;
	}
	case HOME2START_1:
		//ret = home2start(count,data);
		break;
	case HOME2START_2:
		//ret = home2start2(count,data);
		break;
	case MV_FORWARD:
		break;
	default:
		rt_printf("unkonwn cmd\n");

	}

	return ret;
}

int tg(Aris::RT_CONTROL::CMachineData &data,Aris::Core::RT_MSG &recvMsg,Aris::Core::RT_MSG &sendMsg)
{
	static const int cmdSize=8192;

	static char cmdQueue[10][cmdSize];

	static int currentCmd=0;
	static int cmdNum=0;

	static int count = 0;

	static Aris::RT_CONTROL::CMachineData lastCmdData=data,lastStateData=data;
	static Aris::RT_CONTROL::CMachineData stateData, cmdData;

	stateData=data;
	cmdData=data;

	switch (recvMsg.GetMsgID())
	{
	case 0:
		recvMsg.Paste(cmdQueue[(currentCmd+cmdNum)%10]);
		++cmdNum;
		break;
	default:
		break;
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
			rt_printf("the cmd is:%d in count:%d\n",cmdData.motorsCommands[0],count);
		}
	}
	else
	{
		cmdData=lastCmdData;
	}


	data=cmdData;

	lastStateData=stateData;
	lastCmdData=cmdData;

	return 0;
}
