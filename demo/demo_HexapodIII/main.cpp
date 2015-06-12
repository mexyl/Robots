#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>
#include <Aris_ExpCal.h>

#include <map>

#include "HexapodIII.h"
#include "HexapodIV.h"
#include "Robot_Base.h"

using namespace std;
using namespace Robots;

ROBOT robot;

using namespace Aris::Core;
using namespace Aris::DynKer;

ROBOT_III rbt;
//Robots::ROBOT_IV rbt;

int main()
{
#ifdef PLATFORM_IS_WINDOWS
	robot.LoadXML("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
	rbt.LoadXML("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
	rbt.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif

	

	double pIn[18] = { 0.7, 0.71, 0.72
		, 0.73, 0.74, 0.75
		, 0.76, 0.77, 0.78
		, 0.79, 0.80, 0.81
		, 0.82, 0.83, 0.84
		, 0.85, 0.86, 0.87 };
	double pEE[18];
	double vIn[18] = { 0.31, 0.32, 0.33
		, 0.34, 0.35, 0.36
		, 0.37, 0.38, 0.39
		, 0.40, 0.41, 0.42
		, 0.43, 0.44, 0.45
		, 0.46, 0.47, 0.48 };
	double vEE[18];
	double aIn[18] = { 0.11, 0.12, 0.13
		, 0.14, 0.15, 0.16
		, 0.17, 0.18, 0.19
		, 0.20, 0.21, 0.22
		, 0.23, 0.24, 0.25
		, 0.26, 0.27, 0.28 };
	double aEE[18];

	double bodyEp[6] = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

	const char *RelativeMarker = "G";

	robot.SetPin(pIn,bodyEp);
	rbt.SetPin(pIn,bodyEp);

	rbt.GetPee(pEE, RelativeMarker);
	dsp(pEE, 18, 1);
	robot.GetPee(pEE, RelativeMarker);
	dsp(pEE, 18, 1);

	/*robot.SetVin(vIn, bodyEp);
	rbt.SetVin(vIn, bodyEp);

	rbt.GetVee(vEE, RelativeMarker);
	dsp(vEE, 18, 1);
	robot.GetVee(vEE, RelativeMarker);
	dsp(vEE, 18, 1);

	robot.SetAin(aIn, bodyEp);
	rbt.SetAin(aIn, bodyEp);

	rbt.GetAee(aEE, RelativeMarker);
	dsp(aEE, 18, 1);
	robot.GetAee(aEE, RelativeMarker);
	dsp(aEE, 18, 1);*/

	memset(pIn, 0, sizeof(pIn));
	memset(vIn, 0, sizeof(vIn));
	memset(aIn, 0, sizeof(aIn));


	robot.SetPee(pEE, bodyEp, RelativeMarker);
	rbt.SetPee(pEE, bodyEp, RelativeMarker);
	rbt.GetPin(pIn);
	dsp(pIn, 18, 1);
	robot.GetPin(pIn);
	dsp(pIn, 18, 1);

	//robot.SetVee(vEE, bodyEp, RelativeMarker);
	//rbt.SetVee(vEE, bodyEp, RelativeMarker);
	//rbt.GetVin(vIn);
	//dsp(vIn, 18, 1);
	//robot.GetVin(vIn);
	//dsp(vIn, 18, 1);

	//robot.SetAee(aEE, bodyEp, RelativeMarker);
	//rbt.SetAee(aEE, bodyEp, RelativeMarker);
	//rbt.GetAin(aIn);
	//dsp(aIn, 18, 1);
	//robot.GetAin(aIn);
	//dsp(aIn, 18, 1);





	cout << "finished" << endl;





	char aaa;
	cin>>aaa;
	return 0;
}

