#include <Aris_Control.h>
#include <Robot_Gait.h>
#include <HexapodIII.h>
#include <string>
#include <map>


enum ROBOT_CMD_ID
{
	ENABLE,
	DISABLE,
	HOME,
	RESET_ORIGIN,
	BEGIN,
	WALK,

	ROBOT_CMD_COUNT
};


extern Robots::ROBOT_III robot;

struct MOTOR_PARAM:public Robots::GAIT_PARAM_BASE
{
	int motorNum;
	int motorID[18];
};



void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string,std::string> &params);
void GenerateCmdMsg(const std::string &cmd, const std::map<std::string,std::string> &params, Aris::Core::MSG &msg);
int tg(Aris::RT_CONTROL::CMachineData &,Aris::Core::RT_MSG &,Aris::Core::RT_MSG &);
