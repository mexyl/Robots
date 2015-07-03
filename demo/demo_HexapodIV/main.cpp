#include "HexapodIV.h"
#include <Aris_Plan.h>

using namespace std;
using namespace Aris::DynKer;
using namespace Aris::Plan;
using namespace Robots;

double d = 0.4;
double h = 0.05;
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

Robots::ROBOT_IV rbt;

int move_forward_acc(unsigned count,double *pIn, double *pEE, double *pBodyEp)
{
	memset(pBodyEp, 0, sizeof(double) * 6);
	memset(pIn, 0, sizeof(double) * 18);
	memcpy(pEE, iniEE, sizeof(double) * 18);

	double alpha = -(PI / 2)*cos(PI * (count+1) / totalCount) + PI / 2;

	pEE[0] = iniEE[0] - (d / 4)*cos(alpha) + d / 4;
	pEE[2] = iniEE[2] + h*sin(alpha);
	pEE[6] = iniEE[6] - (d / 4)*cos(alpha) + d / 4;
	pEE[8] = iniEE[8] + h*sin(alpha);
	pEE[12] = iniEE[12] - (d / 4)*cos(alpha) + d / 4;
	pEE[14] = iniEE[14] + h*sin(alpha);

	pBodyEp[3] = acc_even(totalCount, count+1) *d / 4;

	rbt.SetPee(pEE, pBodyEp);
	rbt.GetPin(pIn);

	return totalCount-count - 1;
}

int move_forward_dec(unsigned count, double *pIn, double *pEE, double *pBodyEp)
{
	memset(pBodyEp, 0, sizeof(double) * 6);
	memset(pIn, 0, sizeof(double) * 18);
	memcpy(pEE, iniEE, sizeof(double) * 18);

	double alpha = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

	pEE[0] = iniEE[0] + d / 2;
	pEE[6] = iniEE[6] + d / 2;
	pEE[12] = iniEE[12] + d / 2;


	pEE[3] = iniEE[3] - (d / 4)*cos(alpha) + d / 4;
	pEE[5] = iniEE[5] + h*sin(alpha);
	pEE[9] = iniEE[9] - (d / 4)*cos(alpha) + d / 4;
	pEE[11] = iniEE[11] + h*sin(alpha);
	pEE[15] = iniEE[15] - (d / 4)*cos(alpha) + d / 4;
	pEE[17] = iniEE[17] + h*sin(alpha);

	pBodyEp[3] = dec_even(totalCount, (count + 1)) *d / 4 + d / 4;

	rbt.SetPee(pEE, pBodyEp);
	rbt.GetPin(pIn);

	return totalCount - count - 1;
}

int home2start1(unsigned count, double *pIn, double *pEE, double *pBodyEp)
{
	const unsigned period1 = 1000;
	const unsigned period2 = 1000;
	
	memset(pBodyEp, 0, sizeof(double) * 6);
	memset(pIn, 0, sizeof(double) * 18);
	memcpy(pEE, homeEE, sizeof(double) * 18);

	double locHeight = -0.55;


	if (count < period1)
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1) / period1) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[0] = homeEE[0] * (cos(alpha) + 1) / 2 - iniEE[0] * (cos(alpha) - 1) / 2;
		pEE[6] = homeEE[6] * (cos(alpha) + 1) / 2 - iniEE[6] * (cos(alpha) - 1) / 2;
		pEE[12] = homeEE[12] * (cos(alpha) + 1) / 2 - iniEE[12] * (cos(alpha) - 1) / 2;

		pEE[2] = homeEE[2] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[8] = homeEE[8] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[14] = homeEE[14] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
	}
	else
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1 - period1) / period2) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[0] = iniEE[0];
		pEE[6] = iniEE[6];
		pEE[12] = iniEE[12];

		pEE[2] = locHeight * (cos(alpha) + 1) / 2 - iniEE[2] * (cos(alpha) - 1) / 2;
		pEE[8] = locHeight * (cos(alpha) + 1) / 2 - iniEE[8] * (cos(alpha) - 1) / 2;
		pEE[14] = locHeight * (cos(alpha) + 1) / 2 - iniEE[14] * (cos(alpha) - 1) / 2;
	}

	rbt.SetPee(pEE, pBodyEp);
	rbt.GetPin(pIn);

	return period1 + period2 - count -1;
}

int home2start2(unsigned count, double *pIn, double *pEE, double *pBodyEp)
{
	const unsigned period1 = 1000;
	const unsigned period2 = 1000;

	memset(pBodyEp, 0, sizeof(double) * 6);
	memset(pIn, 0, sizeof(double) * 18);
	memcpy(pEE, homeEE, sizeof(double) * 18);

	double locHeight = -0.55;


	if (count < period1)
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1) / period1) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[3] = homeEE[3] * (cos(alpha) + 1) / 2 - iniEE[3] * (cos(alpha) - 1) / 2;
		pEE[9] = homeEE[9] * (cos(alpha) + 1) / 2 - iniEE[9] * (cos(alpha) - 1) / 2;
		pEE[15] = homeEE[15] * (cos(alpha) + 1) / 2 - iniEE[15] * (cos(alpha) - 1) / 2;

		pEE[5] = homeEE[5] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[11] = homeEE[11] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
		pEE[17] = homeEE[17] * (cos(alpha) + 1) / 2 - locHeight * (cos(alpha) - 1) / 2;
	}
	else
	{
		double alpha = -(PI / 2)*cos(PI * (count + 1 - period1) / period2) + PI / 2;//0 to PI,cos(alpha)is 1 to -1
		pEE[3] = iniEE[3];
		pEE[9] = iniEE[9];
		pEE[15] = iniEE[15];

		pEE[5] = locHeight * (cos(alpha) + 1) / 2 - iniEE[5] * (cos(alpha) - 1) / 2;
		pEE[11] = locHeight * (cos(alpha) + 1) / 2 - iniEE[11] * (cos(alpha) - 1) / 2;
		pEE[17] = locHeight * (cos(alpha) + 1) / 2 - iniEE[17] * (cos(alpha) - 1) / 2;
	}

	rbt.SetPee(pEE, pBodyEp);
	rbt.GetPin(pIn);

	return period1 + period2 - count - 1;
}






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
		move_forward_acc(i, pIn_acc[i], pEE_acc[i], pBodyEp_acc[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_acc.txt", *pIn_acc, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_acc.txt", *pEE_acc, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_acc.txt", *pBodyEp_acc, totalCount, 6);

	double pEE_dec[totalCount][18], pIn_dec[totalCount][18], pBodyEp_dec[totalCount][6];

	for (unsigned i = 0; i < totalCount; ++i)
	{
		move_forward_dec(i, pIn_dec[i], pEE_dec[i], pBodyEp_dec[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_dec.txt", *pIn_dec, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_dec.txt", *pEE_dec, totalCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_dec.txt", *pBodyEp_dec, totalCount, 6);


	const int home2startCount = 2000;
	double pEE_Mat[home2startCount][18], pIn_Mat[home2startCount][18], pBodyEp_Mat[home2startCount][6];

	for (unsigned i = 0; i < home2startCount; ++i)
	{
		home2start2(i, pIn_Mat[i], pEE_Mat[i], pBodyEp_Mat[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\pIn_Mat.txt", *pIn_Mat, home2startCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pEE_Mat.txt", *pEE_Mat, home2startCount, 18);
	dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_Mat.txt", *pBodyEp_Mat, home2startCount, 6);


	cout << "finished" << endl;





	char aaa;
	cin>>aaa;
	return 0;
}

