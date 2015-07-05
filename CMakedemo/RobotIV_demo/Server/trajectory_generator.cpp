#include "trajectory_generator.h"
#include <cstring>
#include <Aris_Plan.h>
#include <Robot_Gait.h>

using namespace std;


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

const int motorNum = 18;

const double d = 0.3;
const double h = 0.06;
//const unsigned totalCount = 1000;

const double iniEE[18]=
{
	-0.396, 0.357, -0.65,
	-0.539, 0, -0.65,
	-0.396, -0.357, -0.65,
	0.396, 0.357, -0.65,
	0.539, 0, -0.65,
	0.396, -0.357, -0.65,
};

const double homeEE[18] =
{
	-0.1279292015817467 - 0.396, 0.357, -0.4902687415900912,
	-0.1279292015817467 - 0.539, 0, -0.4902687415900912,
	-0.1279292015817467 - 0.396, -0.357, -0.4902687415900912,
	0.1279292015817467 + 0.396, 0.357, -0.4902687415900912,
	0.1279292015817467 + 0.539, 0, -0.4902687415900912,
	0.1279292015817467 + 0.396, -0.357, -0.4902687415900912,
};

int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0,
};

double meter2count = 1/0.005*1.5*65536;

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

	for(unsigned i=0;i<9;++i)
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
int home2(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
	static unsigned homeMotors[9]={3,4,5,9,10,11,15,16,17};

	bool isAllHomed=true;

	for(unsigned i=0;i<9;++i)
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

int walk_acc(unsigned count , Aris::RT_CONTROL::CMachineData &data,const ROBOT_CMD &cmd)
{
	double pBodyEp[6];
	double pIn[18];
	double pEE[18];

	unsigned ret = Robots::walk_acc(&robot,count
			,cmd.param[0].toInt
			,iniEE
			,cmd.param[3].toDouble
			,cmd.param[4].toDouble
			,cmd.param[5].toDouble
			,cmd.param[6].toDouble
			,cmd.param[1].toChar
			,cmd.param[2].toChar
			,pIn,pEE,pBodyEp);


	for(unsigned i=0;i<motorNum;++i)
	{
		data.commandData[i].Position=pIn[i]*meter2count;
	}

	return ret+cmd.param[0].toInt;
}
int walk_dec(unsigned count , Aris::RT_CONTROL::CMachineData &data,const ROBOT_CMD &cmd)
{
	double pBodyEp[6];
	double pIn[18];
	double pEE[18];

	unsigned ret = Robots::walk_dec(&robot,count
			,cmd.param[0].toInt
			,iniEE
			,cmd.param[3].toDouble
			,cmd.param[4].toDouble
			,cmd.param[5].toDouble
			,cmd.param[6].toDouble
			,cmd.param[1].toChar
			,cmd.param[2].toChar
			,pIn,pEE,pBodyEp);

	for(unsigned i=0;i<motorNum;++i)
	{
		data.commandData[i].Position=pIn[i]*meter2count;
	}

	return ret;
}
int walk_const(unsigned count , Aris::RT_CONTROL::CMachineData &data,const ROBOT_CMD &cmd)
{
	double pBodyEp[6];
	double pIn[18];
	double pEE[18];

	unsigned ret = Robots::walk_const(&robot,count
			,cmd.param[0].toInt
			,iniEE
			,cmd.param[3].toDouble
			,cmd.param[4].toDouble
			,cmd.param[5].toDouble
			,cmd.param[6].toDouble
			,cmd.param[1].toChar
			,cmd.param[2].toChar
			,pIn,pEE,pBodyEp);

	for(unsigned i=0;i<motorNum;++i)
	{
		data.commandData[i].Position=pIn[i]*meter2count;
	}

	return ret+1;
}
int home2start1(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
	static unsigned homeMotors[9]={0,1,2,6,7,8,12,13,14};

	const unsigned period1 = 1000;
	const unsigned period2 = 1000;

	double pBodyEp[6];
	double pIn[18];
	double pEE[18];

	memset(pBodyEp, 0, sizeof(double) * 6);
	memset(pIn, 0, sizeof(double) * 18);
	memcpy(pEE, homeEE, sizeof(double) * 18);

	double locHeight = -0.55;


	if (count < period1)
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1) / period1) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[0] = homeEE[0] * (cos(alpha) + 1) / 2 - iniEE[0] * (cos(alpha) - 1) / 2;
		pEE[6] = homeEE[6] * (cos(alpha) + 1) / 2 - iniEE[6] * (cos(alpha) - 1) / 2;
		pEE[12] = homeEE[12] * (cos(alpha) + 1) / 2 - iniEE[12] * (cos(alpha) - 1) / 2;

		pEE[2] = homeEE[2] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[8] = homeEE[8] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[14] = homeEE[14] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
	}
	else
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1 - period1) / period2) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[0] = iniEE[0];
		pEE[6] = iniEE[6];
		pEE[12] = iniEE[12];

		pEE[2] = locHeight * (cos(alpha) + 1) / 2 - iniEE[2] * (cos(alpha) - 1) / 2;
		pEE[8] = locHeight * (cos(alpha) + 1) / 2 - iniEE[8] * (cos(alpha) - 1) / 2;
		pEE[14] = locHeight * (cos(alpha) + 1) / 2 - iniEE[14] * (cos(alpha) - 1) / 2;
	}

	robot.SetPee(pEE, pBodyEp);
	robot.GetPin(pIn);

	for(unsigned i=0;i<9;++i)
	{
		data.motorsCommands[homeMotors[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
		data.commandData[homeMotors[i]].Position=pIn[homeMotors[i]]*meter2count;
	}

	return period1 + period2 - count -1;
}
int home2start2(unsigned count , Aris::RT_CONTROL::CMachineData &data)
{
	static unsigned homeMotors[9]={3,4,5,9,10,11,15,16,17};

	const unsigned period1 = 1000;
	const unsigned period2 = 1000;

	double pBodyEp[6];
	double pIn[18];
	double pEE[18];

	memset(pBodyEp, 0, sizeof(double) * 6);
	memset(pIn, 0, sizeof(double) * 18);
	memcpy(pEE, homeEE, sizeof(double) * 18);

	double locHeight = -0.55;


	if (count < period1)
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1) / period1) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[3] = homeEE[3] * (cos(alpha) + 1) / 2 - iniEE[3] * (cos(alpha) - 1) / 2;
		pEE[9] = homeEE[9] * (cos(alpha) + 1) / 2 - iniEE[9] * (cos(alpha) - 1) / 2;
		pEE[15] = homeEE[15] * (cos(alpha) + 1) / 2 - iniEE[15] * (cos(alpha) - 1) / 2;

		pEE[5] = homeEE[5] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[11] = homeEE[11] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[17] = homeEE[17] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
	}
	else
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1 - period1) / period2) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[3] = iniEE[3];
		pEE[9] = iniEE[9];
		pEE[15] = iniEE[15];

		pEE[5] = locHeight * (cos(alpha) + 1) / 2 - iniEE[5] * (cos(alpha) - 1) / 2;
		pEE[11] = locHeight * (cos(alpha) + 1) / 2 - iniEE[11] * (cos(alpha) - 1) / 2;
		pEE[17] = locHeight * (cos(alpha) + 1) / 2 - iniEE[17] * (cos(alpha) - 1) / 2;
	}

	robot.SetPee(pEE, pBodyEp);
	robot.GetPin(pIn);

	for(unsigned i=0;i<9;++i)
		{
			data.motorsCommands[homeMotors[i]]=Aris::RT_CONTROL::EMCMD_RUNNING;
			data.commandData[homeMotors[i]].Position=pIn[homeMotors[i]]*meter2count;
		}

	return period1 + period2 - count - 1;
}

int execute_cmd(unsigned count,const ROBOT_CMD &cmd,Aris::RT_CONTROL::CMachineData &data)
{
	int ret;

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
	case HOME_2:
		ret = home2(count,data);
		break;
	case HOME2START_1:
		ret = home2start1(count,data);
		break;
	case HOME2START_2:
		ret = home2start2(count,data);
		break;
	case MV_FORWARD:
		if(count<cmd.param[0].toInt)
		{
			ret=walk_acc(count,data,cmd);
		}
		else if( count < (2*cmd.param[9].toInt-1)*cmd.param[0].toInt )
		{
			ret=walk_const((count+cmd.param[0].toInt)%(2*cmd.param[0].toInt),data,cmd);
		}
		else
		{
			ret=walk_dec(count%cmd.param[0].toInt , data , cmd);
		}

		break;
	default:
		rt_printf("unkonwn cmd\n");
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
			rt_printf("to much cmds, ignore last one\n");
		}

		break;
	case MODIFY_PARAM:
		rt_printf("received modify param msg\n");
		break;
	default:
		//rt_printf("received other msg\n");
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
