#include <Aris_Control.h>
#include <HexapodIV.h>

extern Robots::ROBOT_IV robot;


enum CLIENT_CMD_ID
{
	EXECUTE_CMD,
	MODIFY_PARAM,

	CLIENT_CMD_COUNT
};

enum ROBOT_STATE_ID
{
	INVALID,
	DISABLED,
	ENABLED,
	ENABLED1_HOMED2,
	ENABLED1_STARTED2,
	HOMED1_ENABLED2,
	HOMED1_STARTED2,
	STARTED1_ENABLED2,
	STARTED1_HOMED2,
	STARTED,

	ROBOT_STATE_COUNT
};

enum ROBOT_CMD_ID
{
	ENABLE,
	DISABLE,
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

class ROBOT_STATE_MACHINE
{
public:
	static const ROBOT_STATE_MACHINE robot_state_machine;

private:
	static ROBOT_STATE_ID stateMachine[ROBOT_STATE_COUNT][ROBOT_CMD_COUNT];


	ROBOT_STATE_MACHINE();
	~ROBOT_STATE_MACHINE() = default;

	ROBOT_STATE_MACHINE(const ROBOT_STATE_MACHINE &)=delete;
	ROBOT_STATE_MACHINE(ROBOT_STATE_MACHINE &&)=delete;

public:
	ROBOT_STATE_ID operator()(ROBOT_STATE_ID state, ROBOT_CMD_ID cmd) const
	{
		return ROBOT_STATE_MACHINE::stateMachine[state][cmd];
	};
};

struct ROBOT_CMD
{
	ROBOT_CMD_ID id;
	int paramNum;

	union PARAM
	{
		int toInt;
		float toFload;
		double toDouble;
	};

	PARAM param[10];
};


int tg(Aris::RT_CONTROL::CMachineData &,Aris::Core::RT_MSG &,Aris::Core::RT_MSG &);
