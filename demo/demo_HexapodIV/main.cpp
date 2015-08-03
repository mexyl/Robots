#include "HexapodIV.h"
#include "HexapodIII.h"
#include "Robot_Gait.h"
#include <Aris_Plan.h>

using namespace std;
using namespace Aris::DynKer;
using namespace Aris::Plan;
using namespace Robots;

const unsigned totalCount = 500;

double iniEE[18]=
{
	-0.396, 0.357, -0.65,
	-0.539, 0, -0.65,
	-0.396, -0.357, -0.65,
	0.396, 0.357, -0.65,
	0.539, 0, -0.65,
	0.396, -0.357, -0.65,
};

double homeEE[18] =
{ 
	-0.1279292015817467 - 0.396, 0.357, -0.4902687415900912,
	-0.1279292015817467 - 0.539, 0, -0.4902687415900912,
	-0.1279292015817467 - 0.396, -0.357, -0.4902687415900912,
	0.1279292015817467 + 0.396, 0.357, -0.4902687415900912,
	0.1279292015817467 + 0.539, 0, -0.4902687415900912,
	0.1279292015817467 + 0.396, -0.357, -0.4902687415900912,
};

double firstEE[18] =
{
	 - 0.396, 0.357, -0.55,
	 - 0.539, 0, -0.55,
	 - 0.396, -0.357, -0.55,
	 + 0.396, 0.357, -0.55,
	 + 0.539, 0, -0.55,
	 + 0.396, -0.357, -0.55,
};

//Robots::ROBOT_IV rbt;
Robots::ROBOT_III rbt;


int walk(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam)
{
	static double lastPee[18];
	static double lastPbody[6];

	const Robots::WALK_PARAM *pWP = static_cast<const Robots::WALK_PARAM *>(pParam);

	if (pParam->count < pWP->totalCount)
	{
		walkAcc(pRobot, pParam);
	}
	else
	{
		Robots::WALK_PARAM param2 = *pWP;
		param2.count = pWP->count - pWP->totalCount;

		memcpy(param2.beginPee, lastPee, sizeof(lastPee));
		memcpy(param2.beginBodyPE, lastPbody, sizeof(lastPbody));

		walkDec(pRobot, &param2);
	}

	if (pParam->count % 100 == 0)
	{
		double pEE[18];
		double pBody[18];
		pRobot->GetPee(pEE, "G");
		pRobot->GetBodyPe(pBody, "313");
	}


	if (pParam->count == pWP->totalCount - 1)
	{
		pRobot->GetPee(lastPee, "G");
		pRobot->GetBodyPe(lastPbody, "313");

		double *pEE = lastPee;
		double *pBody = lastPbody;
	}
	return 2 * pWP->totalCount - pWP->count - 1;
}

int main()
{
	rbt.LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");

	double beginPee[18]
	{
		-0.3,-0.85,-0.65,
		-0.45,-0.85,0,
		-0.3,-0.85,0.65,
		0.3,-0.85,-0.65,
		0.45,-0.85,0,
		0.3,-0.85,0.65,
	};

	Robots::WALK_PARAM param;
	param.alpha = 0;
	param.beta = 0;
	param.h = 0.05;
	param.d = 0.8;
	param.totalCount = 2000;
	param.n = 1;
	param.upDirection = 2;
	param.walkDirection = -3;

	std::copy_n(beginPee, 18, param.beginPee);
	std::fill_n(param.beginBodyPE, 6, 0);

	const int totalCount = 8000;
	double pEE_Mat[totalCount][18], pIn_Mat[totalCount][18], pBodyEp_Mat[totalCount][6];

	for (int j = 0; j < 1; ++j)
	{
		for (unsigned i = 0; i < 4000; ++i)
		{
			param.count = i;
			walk(&rbt, &param);
			rbt.GetPee(pEE_Mat[i]);
			rbt.GetPin(pIn_Mat[i]);
			rbt.GetBodyPe(pBodyEp_Mat[i]);

		}

		rbt.GetPee(param.beginPee);
		rbt.GetBodyPe(param.beginBodyPE);
	}

	rbt.GetPee(param.beginPee);
	rbt.GetBodyPe(param.beginBodyPE);

	param.alpha = 0;
	param.beta = 0;
	param.h = 0.05;
	param.d = 0.8;
	param.walkDirection = -3;

	for (unsigned i = 0; i < 4000; ++i)
	{
		param.count = i;
		walk(&rbt, &param);
		rbt.GetPee(pEE_Mat[i+4000]);
		rbt.GetPin(pIn_Mat[i + 4000]);
		rbt.GetBodyPe(pBodyEp_Mat[i + 4000]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_Mat.txt", *pIn_Mat, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_Mat.txt", *pEE_Mat, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBody_Mat.txt", *pBodyEp_Mat, totalCount, 6);








	cout << "finished" << endl;


	char aaa;
	cin>>aaa;
	return 0;
}

