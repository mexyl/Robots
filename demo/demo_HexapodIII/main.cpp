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

	
	/*
	{
		rbt.pLF->pSf->Activate();
		rbt.pLF->pM1->Activate();
		rbt.pLF->pM2->Activate();
		rbt.pLF->pM3->Activate();
		rbt.pLF->pF1->Deactivate();
		rbt.pLF->pF2->Deactivate();
		rbt.pLF->pF3->Deactivate();
		rbt.pLF->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLF->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLF->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pLR->pSf->Activate();
		rbt.pLR->pM1->Activate();
		rbt.pLR->pM2->Activate();
		rbt.pLR->pM3->Activate();
		rbt.pLR->pF1->Deactivate();
		rbt.pLR->pF2->Deactivate();
		rbt.pLR->pF3->Deactivate();
		rbt.pLR->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLR->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLR->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pRM->pSf->Activate();
		rbt.pRM->pM1->Activate();
		rbt.pRM->pM2->Activate();
		rbt.pRM->pM3->Activate();
		rbt.pRM->pF1->Deactivate();
		rbt.pRM->pF2->Deactivate();
		rbt.pRM->pF3->Deactivate();
		rbt.pRM->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRM->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRM->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pLM->pSf->Deactivate();
		rbt.pLM->pM1->Deactivate();
		rbt.pLM->pM2->Deactivate();
		rbt.pLM->pM3->Deactivate();
		rbt.pLM->pF1->Activate();
		rbt.pLM->pF2->Activate();
		rbt.pLM->pF3->Activate();
		rbt.pLM->pM1->SetMode(MOTION_BASE::FCE_CONTROL);
		rbt.pLM->pM2->SetMode(MOTION_BASE::FCE_CONTROL);
		rbt.pLM->pM3->SetMode(MOTION_BASE::FCE_CONTROL);

		rbt.pRF->pSf->Deactivate();
		rbt.pRF->pM1->Deactivate();
		rbt.pRF->pM2->Deactivate();
		rbt.pRF->pM3->Deactivate();
		rbt.pRF->pF1->Activate();
		rbt.pRF->pF2->Activate();
		rbt.pRF->pF3->Activate();
		rbt.pRF->pM1->SetMode(MOTION_BASE::FCE_CONTROL);
		rbt.pRF->pM2->SetMode(MOTION_BASE::FCE_CONTROL);
		rbt.pRF->pM3->SetMode(MOTION_BASE::FCE_CONTROL);

		rbt.pRR->pSf->Deactivate();
		rbt.pRR->pM1->Deactivate();
		rbt.pRR->pM2->Deactivate();
		rbt.pRR->pM3->Deactivate();
		rbt.pRR->pF1->Activate();
		rbt.pRR->pF2->Activate();
		rbt.pRR->pF3->Activate();
		rbt.pRR->pM1->SetMode(MOTION_BASE::FCE_CONTROL);
		rbt.pRR->pM2->SetMode(MOTION_BASE::FCE_CONTROL);
		rbt.pRR->pM3->SetMode(MOTION_BASE::FCE_CONTROL);
	}
	*/
	{
		rbt.pLF->pSf->Activate();
		rbt.pLF->pM1->Deactivate();
		rbt.pLF->pM2->Activate();
		rbt.pLF->pM3->Activate();
		rbt.pLF->pF1->Activate();
		rbt.pLF->pF2->Deactivate();
		rbt.pLF->pF3->Deactivate();
		rbt.pLF->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLF->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLF->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pLR->pSf->Activate();
		rbt.pLR->pM1->Deactivate();
		rbt.pLR->pM2->Activate();
		rbt.pLR->pM3->Activate();
		rbt.pLR->pF1->Activate();
		rbt.pLR->pF2->Deactivate();
		rbt.pLR->pF3->Deactivate();
		rbt.pLR->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLR->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLR->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pRM->pSf->Activate();
		rbt.pRM->pM1->Deactivate();
		rbt.pRM->pM2->Activate();
		rbt.pRM->pM3->Activate();
		rbt.pRM->pF1->Activate();
		rbt.pRM->pF2->Deactivate();
		rbt.pRM->pF3->Deactivate();
		rbt.pRM->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRM->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRM->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pLM->pSf->Deactivate();
		rbt.pLM->pM1->Activate();
		rbt.pLM->pM2->Activate();
		rbt.pLM->pM3->Activate();
		rbt.pLM->pF1->Deactivate();
		rbt.pLM->pF2->Deactivate();
		rbt.pLM->pF3->Deactivate();
		rbt.pLM->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLM->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pLM->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pRF->pSf->Deactivate();
		rbt.pRF->pM1->Activate();
		rbt.pRF->pM2->Activate();
		rbt.pRF->pM3->Activate();
		rbt.pRF->pF1->Deactivate();
		rbt.pRF->pF2->Deactivate();
		rbt.pRF->pF3->Deactivate();
		rbt.pRF->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRF->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRF->pM3->SetMode(MOTION_BASE::POS_CONTROL);

		rbt.pRR->pSf->Deactivate();
		rbt.pRR->pM1->Activate();
		rbt.pRR->pM2->Activate();
		rbt.pRR->pM3->Activate();
		rbt.pRR->pF1->Deactivate();
		rbt.pRR->pF2->Deactivate();
		rbt.pRR->pF3->Deactivate();
		rbt.pRR->pM1->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRR->pM2->SetMode(MOTION_BASE::POS_CONTROL);
		rbt.pRR->pM3->SetMode(MOTION_BASE::POS_CONTROL);
	}









	try
	{
		rbt.FastDyn();
	}
	catch (std::exception &e)
	{
		cout << e.what();
	}
	catch (...)
	{

	}
	
	rbt.GetFin(fIn);
	rbt.SetFin(fIn);
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




	for (int i = 0; i < 1; ++i)
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


	
	rbt.DynPre();
	rbt.DynPrtMtx();
	rbt.Dyn();
	rbt.GetFin(fIn);
	dsp(fIn, 18, 1);

	cout << "finished" << endl;

	double vel[6]{ 1,2,3,4,5,6 };
	double pe[6]{ 0,0,0,-PI/2,0,0 };
	double pm[16];

	s_pe2pm(pe, pm);
	

	double inv_vel[6];
	s_inv_v2v(pm,vel,nullptr,inv_vel);
	//dsp(inv_vel,6,1);


	/*
	Aris::DynKer::SIMULATE_SCRIPT script;
	Robots::Activate024(0, &rbt, &script);
	Robots::Activate135(3000, &rbt, &script);
	script.ScriptDt(10);

	Robots::WALK_PARAM wp;
	std::copy(pEE_G, pEE_G + 18, wp.beginPee);

	rbt.SimulateForwardByAdams("adams", walk, &wp, &script);
	*/

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

