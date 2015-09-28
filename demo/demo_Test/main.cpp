#include <Platform.h>

#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_ExpCal.h>
#include <Aris_Plan.h>
#include <Robot_Type_I.h>

using namespace std;
using namespace Aris::Core;
using namespace Aris::DynKer;
using namespace Aris::Plan;

Robots::ROBOT_III robot;

double eePosIni[6][3] =
{ { -0.3, -0.85, -0.65 }
, { -0.45, -0.85, 0 }
, { -0.3, -0.85, 0.65 }
, { 0.3, -0.85, -0.65 }
, { 0.45, -0.85, 0 }
, { 0.3, -0.85, 0.65 } };

double stepH = 0.04;
double stepD = 1.1;
double totalTime = 3000;
double v = stepD / totalTime / 1000;
int leg_index = 0;

void b_const(double s_in, double *b_out)
{
	b_out[0] = eePosIni[leg_index][0];
	b_out[1] = stepH * sin(s_in) + eePosIni[leg_index][1];
	b_out[2] = -stepD / 2 * cos(s_in) + eePosIni[leg_index][2] + stepD / 4;
}
void g_const(double s_in, double *g_out)
{
	g_out[0] = 0;
	g_out[1] = stepH*cos(s_in);
	g_out[2] = stepD / 2 * sin(s_in);
}
void h_const(double s_in, double *h_out)
{
	h_out[0] = 0;
	h_out[1] = -stepH*sin(s_in);
	h_out[2] = stepD / 2 * cos(s_in);
}

void GetEveryThing(Aris::Plan::FAST_PATH::DATA & data)
{
	double bodyPe[6]{ 0,0,v*data.time,0,0,0 }, bodyVel[6]{ 0,0,v,0,0,0 }, bodyAcc[6]{ 0 };
	double bodyPm[16];
	s_pe2pm(bodyPe, bodyPm);
	robot.pBody->SetPm(bodyPm);
	robot.pBody->SetVel(bodyVel);
	robot.pBody->SetAcc(bodyAcc);

	double pEE[3]{ 0 }, vEE[3]{ 0 };
	
	b_const(data.s, pEE);
	g_const(data.s, data.g);
	h_const(data.s, data.h);
	
	s_daxpy(3, data.s, data.g, 1, vEE, 1);

	robot.pLegs[leg_index]->SetPee(pEE, "G");
	robot.pLegs[leg_index]->SetVee(vEE, "G");

	robot.pLegs[leg_index]->GetJvi(data.Ji, "G");
	robot.pLegs[leg_index]->GetCvi(data.Cv, "G");
	robot.pLegs[leg_index]->GetCai(data.Ca, "G");
}


int main()
{
#ifdef PLATFORM_IS_WINDOWS
	robot.LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	robot.LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
	
	FAST_PATH tg;

	tg.SetMotorLimit(std::vector<FAST_PATH::MOTOR_LIMIT> {3, { 0.23,-0.23,0.8,-0.8 } });
	tg.SetBeginNode({ 0.0, 0.0, 0.0, 0.0, true });
	tg.SetEndNode({ totalTime / 1000.0, PI, 0.0, 0.0, true });
	tg.SetFunction(GetEveryThing);
	tg.Run();






	char aaa;
	std::cin >> aaa;
	return 0;
}

