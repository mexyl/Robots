#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_ExpCal.h>

#include <map>

#include "Hexapod_Robot.h"

using namespace std;
using namespace Hexapod_Robot;

extern ROBOT robot;

using namespace Aris::Core;
using namespace Aris::DynKer;


void test()
{
#define input_test_length 2801

	static double pos[input_test_length][3], vel[input_test_length][3], acc[input_test_length][3];
	double fce[input_test_length][3];

	dlmread("C:\\Users\\yang\\Desktop\\pos_calibrated.txt", *pos);
	dlmread("C:\\Users\\yang\\Desktop\\vel_calibrated.txt", *vel);
	dlmread("C:\\Users\\yang\\Desktop\\acc_calibrated.txt", *acc);
	//dlmread("C:\\Users\\yang\\Desktop\\fce_calibrated.txt", *fce);

	
	double input[18] = { 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9 };
	double bodyep[6] = { 0, 0, 0, 0, 0, 0 };

	robot.SetFixedFeet("", "");

	robot.SetPin(input, bodyep);

	memset(input, 0, sizeof(input));
	robot.SetVin(input, bodyep);
	robot.SetAin(input, bodyep);

	

	for (unsigned i = 0; i < input_test_length; ++i)
	{
		robot.pLF->SetPin(pos[i]);
		robot.pLF->SetVin(vel[i]);
		robot.pLF->SetAin(acc[i]);
		//robot.pLF->SetFin(fce[i]);

		robot.FastDyn();

		robot.pLF->GetFin(fce[i]);
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\test_fce.txt", *fce, input_test_length, 3);
}

void calibrate()
{
	//#define input_calibration_length 29
#define input_calibration_length 2801

	static double pos[input_calibration_length][3], vel[input_calibration_length][3], acc[input_calibration_length][3], fce[input_calibration_length][3];

	dlmread("C:\\Users\\yang\\Desktop\\pos_calibrated.txt", *pos);
	dlmread("C:\\Users\\yang\\Desktop\\vel_calibrated.txt", *vel);
	dlmread("C:\\Users\\yang\\Desktop\\acc_calibrated.txt", *acc);
	dlmread("C:\\Users\\yang\\Desktop\\fce_calibrated.txt", *fce);


	double input[18] = { 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9 };
	double bodyep[6] = { 0, 0, 0, 0, 0, 0 };

	robot.SetFixedFeet("12345", "45abgh");

	robot.SetPin(input, bodyep);

	memset(input, 0, sizeof(input));
	robot.SetVin(input, bodyep);
	robot.SetAin(input, bodyep);

	double *clb_d, *clb_b;
	unsigned m, n;

	robot.ForEachMotion([](MOTION *m)
	{
		double f = 0;
		m->SetF_m(&f);
	});

	robot.ClbEqnTo(clb_d, clb_b, m, n);


	//#define clb_num 29
#define clb_num 2801

	MATRIX clb_d_mat, clb_b_mat;
	clb_d_mat.Resize(clb_num * 3, n);
	clb_b_mat.Resize(clb_num * 3, 1);

	for (unsigned i = 0; i < clb_num; ++i)
	{
		robot.pLF->SetPin(pos[i]);
		robot.pLF->SetVin(vel[i]);
		robot.pLF->SetAin(acc[i]);
		robot.pLF->SetFin(fce[i]);

		robot.ClbEqnTo(clb_d, clb_b, m, n);

		memcpy(&clb_d_mat(i * 3, 0), clb_d, 3 * n * sizeof(double));
		memcpy(&clb_b_mat(i * 3, 0), clb_b, 3 * sizeof(double));



		if (i % 100 == 0)
		{
			cout << i << endl;
		}
	}

	dlmwrite("C:\\Users\\yang\\Desktop\\clb_d.txt", clb_d_mat.Data(), clb_d_mat.RowNum(), clb_d_mat.ColNum());
	dlmwrite("C:\\Users\\yang\\Desktop\\clb_b.txt", clb_b_mat.Data(), clb_b_mat.RowNum(), clb_b_mat.ColNum());
}

int main()
{
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


	test();



	cout << "finished" << endl;





	char aaa;
	cin>>aaa;
	return 0;
}

