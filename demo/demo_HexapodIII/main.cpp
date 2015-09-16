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
	double pIn[18], vIn[18], aIn[18], fIn[18];
	double pEE_G[18] =
	{
		-0.4, -0.75, -0.7,
		-0.5, -0.75, 0,
		-0.4, -0.75, 0.7,
		 0.4, -0.75, -0.7,
		 0.5, -0.75, 0,
		 0.4, -0.75, 0.7
	};
	double vEE_G[18] =
	{
		0, 0, 0,
		10, 10, 10,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};
	double aEE_G[18] =
	{
		0, 0, 0,
		20, 10, -10,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};

	double bodyPE[6]{ 0.01,0.02,0.03,0.04,0.05,0.06 };
	//double bodyPE[6]{ 0 };

	double bodyVel[6]{ 0,0,0,0,0,0 };
	//double bodyVel[6]{ 0.9,0.8,0.7,0.6,0.5,0.4 };
	//double bodyVel[6]{ 0 };

	double bodyAcc[6]{ 0,0,0,0,0,0};
	//double bodyAcc[6]{ 0 };

	rbt.SetFixFeet("101010");
	rbt.SetActiveMotion("011111011111011111");

	rbt.SetPee(pEE_G, bodyPE, "G");
	rbt.SetVee(vEE_G, bodyVel, "G");
	rbt.SetAee(aEE_G, bodyAcc, "G");
	rbt.GetPin(pIn);
	rbt.GetVin(vIn);
	rbt.GetAin(aIn);

	clock_t  clockBegin, clockEnd;
	clockBegin = clock();

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
	rbt.GetFin(fIn);
	dsp(fIn, 18, 1);
	
	rbt.Dyn();

	rbt.ForEachMotion([](Aris::DynKer::MOTION_BASE *mot)
	{
		cout << mot->GetMotFce() << endl;
	});


	Robots::ADJUST_PARAM param;
	std::copy_n(pEE_G, 18, &param.targetPee[0][0]);
	std::copy_n(bodyPE, 6, &param.targetBodyPE[0][0]);

	param.targetBodyPE[0][3] += 0.1;
	param.targetPee[0][16] += 0.1;
	
	rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test", Robots::adjust, &param, 10);
	
	rbt.ForEachMotion([](Aris::DynKer::MOTION_BASE *mot)
	{
		cout << mot->GetMotFce() << endl;
	});

	double data[3];
	rbt.pRR->GetPee(data, "G");
	dsp(data, 3,1);
	
	double bodyPe[] = { 0,0,0,0,-0.871,0 };
	rbt.SetPee(pEE_G, bodyPe);
	rbt.GetBodyPe(bodyPe);
	double bodyPe2[] = { 0,0,0,0,-2 * 0.871,0 };
	rbt.SetPee(pEE_G, bodyPe2);
	rbt.GetBodyPe(bodyPe2);
	dsp(bodyPe,1,6);
	



//rbt.SimByAdamsResultAt(155);

	//rbt.GetFinDyn(fIn);
	//dsp(fIn, 18, 1);

	/*
	rbt.ForEachForce([](Aris::DynKer::FORCE_BASE *fce) 
	{
		fce->Deactivate();
	});

	int clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim;
	rbt.ClbPre(clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim);

	Aris::DynKer::MATRIX clb_d, clb_b, clb_gamma;
	clb_d.Resize(clb_dim_m, clb_dim_n);
	clb_b.Resize(clb_dim_m, 1);
	clb_gamma.Resize(gamma_dim + frc_coe_dim, 1);


	rbt.ClbMtx(clb_d.Data(),clb_b.Data());
	rbt.ClbUnk(clb_gamma.Data());

	clb_b.dsp();

	clb_b = clb_d*clb_gamma;

	clb_b.dsp();
	*/
	

	//rbt.SaveAdams("test");
	
	//Aris::DynKer::SIMULATE_SCRIPT script;
	//Robots::Activate024(0, &rbt, &script);
	//Robots::Activate135(3000, &rbt, &script);
	//script.SetDt(10);

	//Robots::WALK_PARAM wp;
	//std::copy(pEE_G, pEE_G + 18, wp.beginPee);

	//rbt.SimByAdams("test", walk, &wp, &script);
	

		
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

