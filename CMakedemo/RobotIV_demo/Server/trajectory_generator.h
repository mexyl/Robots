#include <Aris_Control.h>
#include <HexapodIV.h>
#include "robot_interface.h"

extern Robots::ROBOT_IV robot;

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




int tg(Aris::RT_CONTROL::CMachineData &,Aris::Core::RT_MSG &,Aris::Core::RT_MSG &);
