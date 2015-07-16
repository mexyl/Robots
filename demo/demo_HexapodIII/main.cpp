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

using namespace std;
using namespace Robots;

using namespace Aris::Core;
using namespace Aris::DynKer;

ROBOT_III rbt;

int main()
{
#ifdef PLATFORM_IS_WINDOWS
	rbt.LoadXML("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
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
		0.4, -0.7, 0,
		0.4, -0.7, 0.7
	};

	double bodyPE[6] = { 0.1,0,0,0,0,0 };

	rbt.SetPee(pEE_G, bodyPE, "G");
	rbt.GetPin(pIn);

	dsp(pIn, 18, 1);

	/*Compute inverse velocity kinematics in Body coordinates*/
	double vIn[18];
	double vEE_G[18] =
	{
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};
	double bodyVel[6] = { 0, 0, 0, 0, 0, 0 };

	rbt.SetVee(vEE_G, bodyVel, "G");
	rbt.GetVin(vIn);

	dsp(vIn, 18, 1);

	/*Compute forward acceleration kinematics in Leg coordinates*/
	double aEE_G[18] =
	{
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};
	double aIn[18];

	double bodyAcc[6] = { 0, 0, 0, 0, 0, 0 };

	rbt.SetAee(aEE_G, bodyAcc, "G");
	rbt.GetAin(aIn);
	dsp(aIn, 18, 1);


	rbt.SetPin(pIn);
	rbt.SetVin(vIn);
	rbt.SetAin(aIn);

	/*compute dynamics*/
	double fIn[18];

	rbt.SetFixedFeet("024", "1278de");
	rbt.FastDyn();
	rbt.GetFin(fIn);
	dsp(fIn, 18, 1);

	rbt.SaveAdams("adams.cmd");

	
	rbt.DynPre();
	rbt.DynPrtMtx();
	rbt.Dyn();
	rbt.GetFin(fIn);
	dsp(fIn, 18, 1);

	cout << "finished" << endl;



	char aaa;
	cin >> aaa;
	return 0;
}

