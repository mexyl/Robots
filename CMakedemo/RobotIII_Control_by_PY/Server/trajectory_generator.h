#include <Aris_Control.h>
#include <HexapodIII.h>

extern Robots::ROBOT_III robot;


struct ROBOT_CMD
{
	int id;
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
