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
		robot.LoadXML("D:\\kuaipan\\Program\\Robots\\Hexapod\\Resource\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
		robot.LoadXML("/home/py/KuaiPan/Program/Robots/Hexapod/Resource/HexapodIII.xml");
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

	//robot.SetFixedFeet("12345", "45abgh");
	robot.SetFixedFeet("024", "1278de");

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

	invel[0] = 0;
	invel[1] = 0;
	invel[2] = 0;

	inacc[0] = 0;
	inacc[1] = 0;
	inacc[2] = 0;

	robot.SetVin(invel, bodyvel);
	robot.SetAin(inacc, bodyacc);

	robot.GetAee(feetacc, "G");
	robot.SetAee(feetacc, 0, "G");

	

	robot.FastDynMtxInPrt();
	dsp(robot.result, 18, 1);

	for (int i = 0; i < 18; ++i)
	{
		robot.GetMotion(i)->SetF_m(&robot.result[i]);
		robot.GetMotion(i)->SetA_m(&inacc[i]);
	}

	robot.DynPre();
	robot.DynPrtMtx();
	robot.Dyn();

	dsp(robot.x,9,1,447,0);

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











	/*
	MODEL m;

	double gamma[10] = { 1.3, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	double im[6][6];
	double pm[4][4];
	double ep[6] = { 0, 0, 0, 0.1, 0.5, 0.6 };
	double vel[6] = { 0, 0, 0, 0, 0, 0.16 };
	double acc[6] = { 0, 0, 0, 0, 0, 0.021 };
	double fce[1] = { 1.5 };

	s_gamma2im(gamma, *im);

	s_ep2pm(ep, *pm);

	double ep2[6] = { 0, 0, 0, 0, 0, 0 };

	m.AddPart("PART1", *im, *pm, vel, acc);
	m.pGround->AddMarker("MARKER1", *pm);
	s_ep2pm(ep2, *pm);
	m.GetPart("PART1")->AddMarker("MARKER2", *pm);
	m.AddJoint("P1", JOINT::PRISMATIC, m.pGround->GetMarker("MARKER1"), m.GetPart("PART1")->GetMarker("MARKER2"));
	m.AddMotion("M1",MOTION::LINEAR,MOTION::POS_CONTROL, m.pGround->GetMarker("MARKER1"), m.GetPart("PART1")->GetMarker("MARKER2"));

	m.GetMotion("M1")->SetP_m(&ep[0]);
	m.GetMotion("M1")->SetV_m(&vel[5]);
	m.GetMotion("M1")->SetA_m(&acc[5]);

	m.DynPre();
	m.DynPrtMtx();

	dlmwrite("C:\\Users\\yang\\Desktop\\C.txt", m.C, 12, 12);
	dlmwrite("C:\\Users\\yang\\Desktop\\D.txt", m.D, 24, 24);
	dlmwrite("C:\\Users\\yang\\Desktop\\x.txt", m.x, 24, 1);
	//dsp(m.C, 12, 12);


	m.Dyn();

	cout << *m.GetMotion("M1")->GetF_mPtr() << endl;

	double *clb_d, *clb_b;
	unsigned clb_m, clb_n;
	m.ClbEqnTo(clb_d, clb_b, clb_m, clb_n);

	dlmwrite("C:\\Users\\yang\\Desktop\\clb_d.txt", clb_d, clb_m, clb_n);
	
	
	*/


	double d[1000],b[20];




	char aaa;
	cin>>aaa;
	return 0;
}

