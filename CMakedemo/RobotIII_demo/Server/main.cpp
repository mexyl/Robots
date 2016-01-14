#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_IMU.h>
#include <Aris_Plan.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

using namespace Aris::Core;



struct SimpleWalkParam final :public Robots::GaitParamBase
{
	std::int32_t totalCount{ 3000 };
	std::int32_t n{ 1 };
	double d{ 0.5 };
	double h{ 0.05 };
};
int simpleWalk(Robots::RobotBase * pRobot, const Robots::GaitParamBase * pParam)
{
	auto pSP = static_cast<const SimpleWalkParam*>(pParam);

	static Aris::DynKer::FloatMarker beginBodyMak(pRobot->Ground(), nullptr, "313");
	static double beginEE[18];

	if ((pSP->count % pSP->totalCount) == 0)
	{
		beginBodyMak.SetPrtPm(*pRobot->Body().Pm());
		pRobot->GetPee(beginEE, beginBodyMak);
	}

	/*加速段*/
	if ((pSP->count / pSP->totalCount) == 0)
	{
		double pEE[18], pe[6]{ 0 };
		std::copy_n(beginEE, 18, pEE);

		pe[2] = Aris::Plan::acc_even(pSP->totalCount, pSP->count + 1)*0.25*pSP->d;

		double s = -(PI / 2)*cos(PI * (pSP->count + 1) / pSP->totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = pSP->h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = -pSP->d*cos(s) / 2 + beginEE[i + 2];
		}
	}

	

	double pEE[18];
	
	


	return 2 * pSP->n * pSP->totalCount - pSP->count;
}
Aris::Core::Msg parseSimpleWalk(const std::string &cmd, const std::map<std::string, std::string> &params);





int main()
{
	auto rs = Robots::RobotServer::GetInstance();
	rs->CreateRobot<Robots::RobotTypeI>();
#ifdef WIN32
	rs->LoadXml("/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml");
#endif
#ifdef UNIX
	rs->LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
	rs->AddGait("wk", Robots::walk, Robots::parseWalk);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
	rs->AddGait("ro", Robots::resetOrigin, Robots::parseResetOrigin);
	rs->Start();
	std::cout<<"started"<<std::endl;

	
	
	Aris::Core::RunMsgLoop();

	return 0;
}
