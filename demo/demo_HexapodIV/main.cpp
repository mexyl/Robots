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


int main()
{
	rbt.LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
	
	//const int home2startCount = 2000;
	//double pEE_Mat[home2startCount][18], pIn_Mat[home2startCount][18], pBodyEp_Mat[home2startCount][6];

	//for (unsigned i = 0; i < home2startCount; ++i)
	//{
	//	home2start(&rbt,i, 1000, 1000, homeEE, firstEE, iniEE, pIn_Mat[i], pEE_Mat[i], pBodyEp_Mat[i]);
	//}

	//dlmwrite("C:\\Users\\yang\\Desktop\\pIn_Mat.txt", *pIn_Mat, home2startCount, 18);
	//dlmwrite("C:\\Users\\yang\\Desktop\\pEE_Mat.txt", *pEE_Mat, home2startCount, 18);
	//dlmwrite("C:\\Users\\yang\\Desktop\\pBodyEp_Mat.txt", *pBodyEp_Mat, home2startCount, 6);


	cout << "finished" << endl;


	char aaa;
	cin>>aaa;
	return 0;
}

