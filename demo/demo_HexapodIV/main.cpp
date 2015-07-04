#include "HexapodIV.h"
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

Robots::ROBOT_IV rbt;

int main()
{
	double pIn[18];
	
	/*在地面坐标系下计算反解*/
	double pEE_G[18] = 
	{
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
	};
	
	double bodyEp[6] = { 0,0,0,0,0,0 };
	
	rbt.SetPee(pEE_G, bodyEp, "G");
	rbt.GetPin(pIn);

	//dsp(pIn, 18, 1);

	/*在身体坐标系下计算反解*/
	double pEE_M[18] =
	{
		-0.4, -0.7, -0.7,
		-0.5, -0.7, 0,
		-0.4, -0.7, 0.7,
		0.4, -0.7, -0.7,
		0.5, -0.7, 0,
		0.4, -0.7, 0.7
	};

	//rbt.SetPee(pEE_M, nullptr, "M");
	//rbt.GetPin(pIn);

	//dsp(pIn, 18, 1);

	/*在单腿坐标系下计算反解*/
	double pEE_L[18] =
	{
		0.1279292, 0, -0.4902687,
		0.1279292, 0, -0.4902687,
		0.1279292, 0, -0.4902687,
		0.1279292, 0, -0.4902687,
		0.1279292, 0, -0.4902687,
		0.1279292, 0, -0.4902687,
	};

	rbt.SetPee(pEE_L, nullptr, "L");
	rbt.GetPin(pIn);

	dsp(pIn, 18, 1);



	double pEE_acc[totalCount][18] , pIn_acc[totalCount][18], pBodyEp_acc[totalCount][6];

	for (unsigned i = 0; i < totalCount; ++i)
	{
		walk_acc(&rbt, i, totalCount, iniEE, 0.2, 0.05, "x", "z", pIn_acc[i], pEE_acc[i], pBodyEp_acc[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_acc.txt", *pIn_acc, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_acc.txt", *pEE_acc, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_acc.txt", *pBodyEp_acc, totalCount, 6);

	double pEE_dec[totalCount][18], pIn_dec[totalCount][18], pBodyEp_dec[totalCount][6];

	for (unsigned i = 0; i < totalCount; ++i)
	{
		walk_dec(&rbt, i, totalCount, iniEE, 0.2, 0.05, "x", "z", pIn_dec[i], pEE_dec[i], pBodyEp_dec[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_dec.txt", *pIn_dec, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_dec.txt", *pEE_dec, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_dec.txt", *pBodyEp_dec, totalCount, 6);


	const int home2startCount = 2000;
	double pEE_Mat[home2startCount][18], pIn_Mat[home2startCount][18], pBodyEp_Mat[home2startCount][6];

	for (unsigned i = 0; i < home2startCount; ++i)
	{
		//home2start(&rbt,i, 1000, 1000, homeEE, firstEE, iniEE, pIn_Mat[i], pEE_Mat[i], pBodyEp_Mat[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_Mat.txt", *pIn_Mat, home2startCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_Mat.txt", *pEE_Mat, home2startCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_Mat.txt", *pBodyEp_Mat, home2startCount, 6);


	cout << "finished" << endl;


	char aaa;
	cin>>aaa;
	return 0;
}

