#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


using namespace Aris::Core;



struct SimpleWalkParam final :public Aris::Server::GaitParamBase
{
	std::int32_t totalCount{ 2000 };
	std::int32_t n{ 1 };
	double d{ 0.5 };
	double h{ 0.05 };
};
void ParseSimpleWalk(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)
{
	SimpleWalkParam param;
	msg_out.copyStruct(param);
}
int SimpleWalk(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)
{
	auto &param = static_cast<const SimpleWalkParam&>(plan_param);
	auto &robot = static_cast<Robots::RobotTypeI&>(model);

	static Aris::Dynamic::FloatMarker beginBodyMak(robot.ground(), nullptr, "313");
	static double beginEE[18];

	/*在每次脚着地时更新与身体坐标系重合的位于地面的坐标系*/
	if ((param.count % param.totalCount) == 0)
	{
		beginBodyMak.setPrtPm(*robot.Body().pm());
		beginBodyMak.update();
		robot.GetPee(beginEE, beginBodyMak);
	}

	double Pee[18], Peb[6]{ 0 };
	std::copy_n(beginEE, 18, Pee);

	/*当前相位的count*/
	int count = param.count % param.totalCount;

	if ((param.count / param.totalCount) == 0)/*加速段*/
	{
		Peb[2] = Aris::Dynamic::acc_even(param.totalCount, count + 1)*0.25*param.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / param.totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			Pee[i + 1] = param.h*sin(s) + beginEE[i + 1];
			Pee[i + 2] = param.d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((param.count / param.totalCount) == (param.n * 2 - 1))/*减速段*/
	{
		Peb[2] = Aris::Dynamic::dec_even(param.totalCount, count + 1)*0.25*param.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / param.totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			Pee[i + 1] = param.h*sin(s) + beginEE[i + 1];
			Pee[i + 2] = param.d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((param.count / param.totalCount) % 2 == 1)/*第一匀速段，紧接着加速段*/
	{
		Peb[2] = Aris::Dynamic::even(param.totalCount, count + 1)*0.5*param.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / param.totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			Pee[i + 1] = param.h*sin(s) + beginEE[i + 1];
			Pee[i + 2] = param.d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else/*第二匀速段，后面就是减速段*/
	{
		Peb[2] = Aris::Dynamic::even(param.totalCount, count + 1)*0.5*param.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / param.totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			Pee[i + 1] = param.h*sin(s) + beginEE[i + 1];
			Pee[i + 2] = param.d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}

	robot.SetPeb(Peb, beginBodyMak);
	robot.SetPee(Pee, beginBodyMak);

	return 2 * param.n * param.totalCount - param.count - 1;
}



int main()
{
	auto &rs = Aris::Server::ControlServer::instance();
	
	rs.createModel<Robots::RobotTypeI>();
//#ifdef WIN32
//	rs.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
//#endif
//#ifdef UNIX
//	rs.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
//#endif
#ifdef WIN32
	rs.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
	rs.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml");
#endif

	rs.addCmd("en", Robots::basicParseFunc, nullptr);
	rs.addCmd("ds", Robots::basicParseFunc, nullptr);
	rs.addCmd("hm", Robots::basicParseFunc, nullptr);
	rs.addCmd("rc", Robots::parseRecover, Robots::recover);
	rs.addCmd("sw", ParseSimpleWalk, SimpleWalk);
	rs.addCmd("wk", Robots::parseWalk, Robots::walk);

	rs.open();

	rs.setOnExit([]() {Aris::Core::stopMsgLoop(); });
	Aris::Core::runMsgLoop();

	

	return 0;
}
