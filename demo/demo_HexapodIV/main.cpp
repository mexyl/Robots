#include "HexapodIV.h"
#include <Aris_Plan.h>

using namespace std;
using namespace Aris::DynKer;
using namespace Robots;


int move_forward_acc(unsigned count,double *pIn)
{
	unsigned totalCount = 1000;








}



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
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
		0, 0, -0.65,
	};

	rbt.SetPee(pEE_L, nullptr, "L");
	rbt.GetPin(pIn);

	dsp(pIn, 18, 1);




	cout << "finished" << endl;





	char aaa;
	cin>>aaa;
	return 0;
}

