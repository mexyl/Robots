#include <Platform.h>

#include <iostream>

#include <Aris_Plan.h>

#include "Robot_Type_II.h"
#include "Robot_Type_I.h"
#include "Robot_Gait.h"

double homeEE[18] =
{ 
-0.318791579531186,   -0.719675656557493,   -0.500049789146799,
-0.413084678293599,   -0.719675656557493,    0,
-0.318791579531187,   -0.719675656557493,    0.498125689146798,
0.318791579531186,   -0.719675656557493,   -0.500049789146799,
0.413084678293599,   -0.719675656557493,    0,
0.318791579531187,   -0.719675656557493,    0.498125689146798,
};

double firstEE[18] =
{
	-0.3,-0.75,-0.65,
	-0.45,-0.75,0,
	-0.3,-0.75,0.65,
	0.3,-0.75,-0.65,
	0.45,-0.75,0,
	0.3,-0.75,0.65,
};

double beginEE[18]
{
	-0.3,-0.85,-0.65,
	-0.45,-0.85,0,
	-0.3,-0.85,0.65,
	0.3,-0.85,-0.65,
	0.45,-0.85,0,
	0.3,-0.85,0.65,
};

double beginPe[6]{ 0 };

Robots::ROBOT_TYPE_I rbt;

int main()
{
#ifdef PLATFORM_IS_WINDOWS
	rbt.LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	rbt.LoadXml("/usr/Robots/resource/Robot_Type_I/HexapodVIII.xml");
#endif

	rbt.SetPee(beginEE, beginPe);

	Robots::WALK_PARAM param;
	param.totalCount = 900;
	param.d = 1.1;
	param.h = 0.04;
	param.n = 1;


	rbt.SimByMatlab("C:\\Users\\yang\\Desktop\\pIn_Walk.txt", Robots::walk, &param);


	std::cout << "finished" << std::endl;


	Robots::ROBOT_IV rbt;

	rbt.SetPee(beginEE);

	char aaa;
	std::cin>>aaa;
	return 0;
}

