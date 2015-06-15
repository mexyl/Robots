#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_ExpCal.h>

#include <map>

#include "HexapodIII.h"
#include "HexapodIV.h"
#include "Robot_Base.h"

using namespace std;
using namespace Robots;

ROBOT robot;

using namespace Aris::Core;
using namespace Aris::DynKer;

ROBOT_III rbt;
//Robots::ROBOT_IV rbt;

int main()
{
#ifdef PLATFORM_IS_WINDOWS
	robot.LoadXML("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
	rbt.LoadXML("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
	rbt.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif

	
	
	

	/*Compute inverse position kinematics in Ground coordinates*/
	double pIn[18];
	double pEE_G[18] =
	{
		-0.4, -0.7, -0.7,
		-0.5, -0.7, 0,
		-0.4, -0.7, 0.7,
		0.4, -0.7, -0.7,
		0.5, -0.7, 0,
		0.4, -0.7, 0.7
	};

	double bodyEp[6] = { 0.1, 0, 0, 0, 0.2, 0 };

	rbt.SetPee(pEE_G, bodyEp, "G");
	rbt.GetPin(pIn);

	dsp(pIn, 18, 1);

	/*Compute inverse velocity kinematics in Body coordinates*/
	double vIn[18];
	double vEE_M[18] =
	{
		-0.4, -0.7, -0.7,
		-0.5, -0.7, 0,
		-0.4, -0.7, 0.7,
		0.4, -0.7, -0.7,
		0.5, -0.7, 0,
		0.4, -0.7, 0.7
	};
	double bodyVel[6] = { 0.1, 0, 0, 0, 0, 0 };

	rbt.SetVee(vEE_M, bodyVel, "M");
	rbt.GetVin(vIn);

	dsp(vIn, 18, 1);

	/*Compute forward acceleration kinematics in Leg coordinates*/
	double aEE_L[18];
	double aIn[18]=
	{
		0.1, 0.2, 0.3,
		0.1, 0.2, 0.3,
		0.1, 0.2, 0.3,
		0.1, 0.2, 0.3,
		0.1, 0.2, 0.3,
		0.1, 0.2, 0.3,
	};

	double bodyAcc[6] = { 0, 0, 0, 0.1, 0, 0 };

	rbt.SetAin(aIn, bodyAcc);

	rbt.GetAee(aEE_L,"L");
	dsp(aEE_L, 18, 1);





	cout << "finished" << endl;





	char aaa;
	cin >> aaa;
	return 0;
}

