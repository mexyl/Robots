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
		0.4, -0.7, 0,
		0.4, -0.7, 0.7
	};

	double bodyEp[6] = { 0.1, 0, 0, 0, 0.2, 0 };

	rbt.SetPee(pEE_G, bodyEp, "G");
	rbt.GetPin(pIn);

	dsp(pIn, 18, 1);

	/*Compute inverse velocity kinematics in Body coordinates*/
	double vIn[18];
	double vEE_G[18] =
	{
		0, 0, 0,
		-0.5, -0.7, 0,
		0, 0, 0,
		0.4, -0.7, -0.7,
		0, 0, 0,
		0.4, -0.7, 0.7
	};
	double bodyVel[6] = { 0.1, 0, 0, 0, 0, 0 };

	rbt.SetVee(vEE_G, bodyVel, "G");
	rbt.GetVin(vIn);

	dsp(vIn, 18, 1);

	/*Compute forward acceleration kinematics in Leg coordinates*/
	double aEE_G[18] =
	{
		0, 0, 0,
		-0.5, -0.7, 0,
		0, 0, 0,
		0.4, -0.7, -0.7,
		0, 0, 0,
		0.4, -0.7, 0.7
	};
	double aIn[18];

	double bodyAcc[6] = { 0, 0, 0, 0.1, 0, 0 };

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

	
	rbt.DynPre();
	rbt.DynPrtMtx();
	rbt.Dyn();
	rbt.GetFin(fIn);
	dsp(fIn, 18, 1);

	robot.SetPin(pIn, bodyEp);
	robot.SetVin(vIn, bodyVel);
	robot.SetAin(aIn, bodyAcc);
	//robot.SetPee(pEE_G, bodyEp, "G");
	//robot.SetVee(vEE_G, bodyVel, "G");
	//robot.SetAee(aEE_G, bodyAcc, "G");
	robot.SetFixedFeet("024", "1278de");
	robot.FastDyn();
	robot.GetFin(fIn);
	dsp(fIn, 18, 1);

	robot.DynPre();
	robot.DynPrtMtx();
	robot.Dyn();
	robot.GetFin(fIn);
	dsp(fIn, 18, 1);




	cout << "finished" << endl;


	double pm_G02G[4][4] =
	{
		{ -1, 0, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 0, 1 }
	};

	double ep[6] = { PI, 0, PI, 0, 0, 0 };
	double pm_I2G0[4][4];
	s_ep2pm(ep, *pm_I2G0, "321");
	dsp(*pm_I2G0, 4, 4);

	double pm_R2I[4][4] =
	{
		{ 1, 0, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, -1, 0, 0 },
		{ 0, 0, 0, 1 }
	};


	double pm_R2G0[4][4];

	double pm_R2G[4][4];

	s_pm_dot_pm(*pm_I2G0, *pm_R2I, *pm_R2G0);
	s_pm_dot_pm(*pm_G02G, *pm_R2G0, *pm_R2G);
	dsp(*pm_R2G, 4, 4);








	char aaa;
	cin >> aaa;
	return 0;
}

