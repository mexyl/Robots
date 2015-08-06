#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <iomanip>
#include <cmath>

#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_DynModel.h>
#include <Aris_ExpCal.h>

#include "Robot_Server.h"

#include <map>

#include "Robot_Gait.h"
#include "HexapodIII.h"
#include "HexapodIV.h"

using namespace std;
using namespace Robots;

//using namespace Aris::Core;
using namespace Aris::DynKer;

ROBOT_III rbt;

#include<ctime>

int main()
{

#ifdef PLATFORM_IS_WINDOWS
	rbt.LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	rbt.LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif

	/*Compute inverse position kinematics in Ground coordinates*/
	double pIn[18];
	double pEE_G[18] =
	{
		-0.4, -0.75, -0.7,
		-0.5, -0.75, 0,
		-0.4, -0.75, 0.7,
		0.4, -0.75, -0.7,
		0.5, -0.75, 0,
		0.4, -0.75, 0.7
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
		10, 10, 10,
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
		20, 10, -10,
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



	clock_t  clockBegin, clockEnd;
	clockBegin = clock();





	double homeIn[18]
	{
		0.675784824916295,0.697784816196987,0.697784816196987,
		0.675784824916295,0.697784816196987,0.697784816196987,
		0.675784824916295,0.697784816196987,0.697784816196987,
		0.675784824916295,0.697784816196987,0.697784816196987,
		0.675784824916295,0.697784816196987,0.697784816196987,
		0.675784824916295,0.697784816196987,0.697784816196987,
	};

	double homeBody[6]{ 0 };

	rbt.SetPin(homeIn, homeBody);

	rbt.GetPee(homeIn,"G");

	dlmwrite("homeIn.txt",homeIn, 6, 3);




	for (int i = 0; i < 10000; ++i)
	{
		rbt.SetPin(pIn);
		rbt.SetVin(vIn);
		rbt.SetAin(aIn);
		rbt.SetPee(pEE_G, bodyPE, "G");
		rbt.SetVee(vEE_G, bodyVel, "G");
		rbt.SetAee(aEE_G, bodyAcc, "G");
		rbt.FastDyn();
	}

	clockEnd = clock();
	std::cout << "consumed time is:" << double(clockEnd - clockBegin)/ CLOCKS_PER_SEC << std::endl;
	




	cout << "finished" << endl;






	rbt.SaveAdams("adams.cmd");

	
	rbt.DynPre();
	rbt.DynPrtMtx();
	rbt.Dyn();
	rbt.GetFin(fIn);
	dsp(fIn, 18, 1);

	cout << "finished" << endl;

	/*
	double pIni = rbt.pLF->pM1->GetP_mPtr()[0];

	double time[] = { 0, 1, 2, 3, 4, 5 };
	double pos[6] = {0};

	for (int i = 0; i < 6;++i)
	{
		pos[i] = pIni+std::sin(time[i])*0.1;
	}

	rbt.pLM->pM1->SetPosAkimaCurve(6, time, pos);
	rbt.SaveAdams("adams");


	Robots::WALK_PARAM param;
	param.alpha = 0.2;
	param.beta = 0.2;
	param.d = 0.3;
	param.h = 0.1;
	param.totalCount = 1000;
	param.walkDirection = -3;
	param.upDirection = 2;
	memcpy(param.beginPee, pEE_G, sizeof(double) * 18);
	memset(param.beginBodyPE, 0, sizeof(double) * 6);

	param.beginBodyPE[2] += 0.1;
	SIMULATE_SCRIPT script;
	
	auto walkFun = [](ROBOT_BASE *pRobot, const GAIT_PARAM_BASE *pBaseParam)
	{
		const WALK_PARAM *param = static_cast<const WALK_PARAM *>(pBaseParam);
		
		if (pBaseParam->count < param->totalCount)
		{
			return int(param->totalCount) + Robots::walkAcc(pRobot, pBaseParam);
		}
		else
		{
			WALK_PARAM param2 = *param;
			param2.count = param->totalCount - 1;
			Robots::walkAcc(pRobot, &param2);
			
			param2.count = pBaseParam->count - param->totalCount;
			pRobot->GetPee(param2.beginPee);
			pRobot->GetBodyPe(param2.beginBodyPE);

			return Robots::walkDec(pRobot, &param2);
		}
	};

	Activate246(1000, &rbt, &script);
	script.ScriptDt(10);

	Activate135(&rbt);
	rbt.SimulateForwardByAdams("robot", walkFun, &param, &script);

	Activate246(&rbt);
	rbt.SimulateForwardByAdams("simulate", walkDec, &param);
	*/
	char aaa;
	cin >> aaa;

	
	return 0;
}

