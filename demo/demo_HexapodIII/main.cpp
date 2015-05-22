#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>

#include "Aris_ExpCal.h"

#include <map>

#include "Hexapod_Robot.h"

using namespace std;
using namespace Hexapod_Robot;

extern ROBOT robot;

using namespace Aris::Core;
using namespace Aris::DynKer;


int main()
{
	cout << "begin" << endl;
	try
	{
#ifdef PLATFORM_IS_WINDOWS
		robot.LoadXML("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
		robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
	}
	catch (std::logic_error &e)
	{
		cout << e.what();
		abort();
	}
	cout << "end" << endl;

	robot.SaveSnapshotXML("C:\\Users\\yang\\Desktop\\HexapodIII_1.xml");

	double eePos[6][3] =
	{ { -0.3, -0.85, -0.65 }
	, { -0.45, -0.85, 0 }
	, { -0.3, -0.85, 0.65 }
	, { 0.3, -0.85, -0.65 }
	, { 0.45, -0.85, 0 }
	, { 0.3, -0.85, 0.65 } };

	double inPos[18];

	robot.SetPee(*eePos);
	robot.GetPin(inPos);
	robot.GetPee(*eePos);

	dsp(*eePos, 6, 3);


	double bodypos[6] = { 0, 0, 0, 0, 0, 0 };
	//double bodypos[6] = { 0.1, 0.02, 0.001, 0.005, 0.03, 0.01 };
	//'double bodyvel[6] = { 0.1, 0.02, 0.01, 0.1, 0.04, 0.35 };
	double bodyvel[6] = { 0, 0, 0, 0, 0, 0 };
	//double bodyacc[6] = { 0.1, 0.03, 0.12, 0.15, 0.32, 0.1 };
	double bodyacc[6] = { 0, 0, 0, 0, 0, 0 };
	double feetpos[18] = { -0.3, -0.85, -0.65,
		- 0.45, -0.85, 0,
		- 0.3, -0.85, 0.65,
		0.3, -0.85, -0.65,
		0.45, -0.85, 0,
		0.3, -0.85, 0.65 };
	double feetvel[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double feetacc[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double invel[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double inacc[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	robot.SetFixedFeet("12345", "45abgh");
	//robot.SetFixedFeet("024", "1278de");

	robot.SetPee(feetpos, bodypos, "G");
	robot.SetVee(feetvel, bodyvel, "G");
	robot.SetAee(feetacc, bodyacc, "G");

	robot.GetVin(invel);
	robot.GetAin(inacc);

	invel[0] = 0.1;
	invel[1] = 0.12;
	invel[2] = -0.13;

	inacc[0] = 0.1;
	inacc[1] = 0.2;
	inacc[2] = 0.3;

	//invel[0] = 0;
	//invel[1] = 0;
	//invel[2] = 0;

	//inacc[0] = 0;
	//inacc[1] = 0;
	//inacc[2] = 0;

	robot.SetVin(invel, bodyvel);
	robot.SetAin(inacc, bodyacc);

	robot.GetAee(feetacc, "G");
	robot.SetAee(feetacc, 0, "G");

	robot.FastDynMtxInPrt();

	for (int i = 0; i < 18; ++i)
	{
		cout << *robot.GetMotion(i)->GetF_mPtr() << endl;
	}
	cout << endl;

	robot.DynPre();
	robot.DynPrtMtx();
	robot.Dyn();

	for (int i = 0; i < 18; ++i)
	{
		cout << *robot.GetMotion(i)->GetF_mPtr() << endl;
	}
	cout << endl;

	robot.SaveAdams("C:\\Users\\yang\\Desktop\\Hexapod.cmd");


	double *clb_d, *clb_b;
	unsigned clb_m, clb_n;
	robot.ClbEqnTo(clb_d, clb_b, clb_m, clb_n);

	dlmwrite("C:\\Users\\yang\\Desktop\\clb_d.txt", clb_d, clb_m, clb_n);
	dlmwrite("C:\\Users\\yang\\Desktop\\clb_b.txt", clb_b, clb_m, 1);

	MATRIX gamma(robot.GetPartNum()*10,1);
	robot.ForEachPart([&gamma](const PART* p)
	{
		double locGamma[10];
		s_im2gamma(p->GetPrtImPtr(), locGamma);

		memcpy(&gamma(p->GetID()*10, 0), locGamma, sizeof(locGamma));
	});

	dlmwrite("C:\\Users\\yang\\Desktop\\gamma.txt",gamma.Data(), robot.GetPartNum() * 10, 1);







	double d[1000],b[20];




	char aaa;
	cin>>aaa;
	return 0;
}

