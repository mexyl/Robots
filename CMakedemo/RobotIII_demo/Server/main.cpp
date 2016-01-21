#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris_core.h>
#include <aris_message.h>
#include <aris_imu.h>
#include <aris_plan.h>
#include <aris_control_server.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

using namespace Aris::Core;

void ParseFunc(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out) 
{
	Aris::BasicFunctionParam param;
	
	for (auto &i : params)
	{
		if (i.first == "all")
		{
			std::fill_n(param.isMotorActive, 18, true);
		}
		else if (i.first == "first")
		{
			std::fill_n(param.isMotorActive, 18, false);
			std::fill_n(param.isMotorActive + 0, 3, true);
			std::fill_n(param.isMotorActive + 6, 3, true);
			std::fill_n(param.isMotorActive + 12, 3, true);
		}
		else if (i.first == "second")
		{
			std::fill_n(param.isMotorActive, 18, false);
			std::fill_n(param.isMotorActive + 3, 3, true);
			std::fill_n(param.isMotorActive + 9, 3, true);
			std::fill_n(param.isMotorActive + 15, 3, true);
		}
		else if (i.first == "motor")
		{
			std::fill_n(param.isMotorActive, 18, false);
			int id = { stoi(i.second) };
			param.isMotorActive[id] = true;
		}
	}

	msg_out.CopyStruct(param);
}

struct SimpleWalkParam final :public Aris::DynKer::PlanParamBase
{
	double beginPee[18]{ 0 };
	double beginPeb[6]{ 0 };
	std::int32_t totalCount{ 500 };
	std::int32_t n{ 1 };
	double d{ 0.5 };
	double h{ 0.05 };
};
void ParseSimpleWalk(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)
{
	SimpleWalkParam param;
	msg_out.CopyStruct(param);
}
int SimpleWalk(Aris::DynKer::ModelBase &model, const Aris::DynKer::PlanParamBase & param)
{
	auto &sp = static_cast<const SimpleWalkParam&>(param);
	auto &robot = static_cast<Robots::RobotTypeI&>(model);

	static Aris::DynKer::FloatMarker beginBodyMak(robot.Ground(), nullptr, "313");
	static double beginEE[18];

	/*在每次脚着地时更新与身体坐标系重合的位于地面的坐标系*/
	if ((sp.count % sp.totalCount) == 0)
	{
		beginBodyMak.SetPrtPm(*robot.Body().Pm());
		beginBodyMak.Update();
		robot.GetPee(beginEE, beginBodyMak);
	}

	double pEE[18], pe[6]{ 0 };
	std::copy_n(beginEE, 18, pEE);

	/*当前相位的count*/
	int count = sp.count % sp.totalCount;

	if ((sp.count / sp.totalCount) == 0)/*加速段*/
	{
		pe[2] = Aris::Plan::acc_even(sp.totalCount, count + 1)*0.25*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((sp.count / sp.totalCount) == (sp.n * 2 - 1))/*减速段*/
	{
		pe[2] = Aris::Plan::dec_even(sp.totalCount, count + 1)*0.25*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((sp.count / sp.totalCount) % 2 == 1)/*第一匀速段，紧接着加速段*/
	{
		pe[2] = Aris::Plan::even(sp.totalCount, count + 1)*0.5*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else/*第二匀速段，后面就是减速段*/
	{
		pe[2] = Aris::Plan::even(sp.totalCount, count + 1)*0.5*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}

	robot.SetPeb(pe, beginBodyMak);
	robot.SetPee(pEE, beginBodyMak);

	return 2 * sp.n * sp.totalCount - sp.count - 1;
}

struct RecoverParam final :public Aris::DynKer::PlanParamBase
{
	double beginPee[18]{ 0 };
	double beginPeb[6]{ 0 };
	std::int32_t totalCount{ 500 };
	std::int32_t n{ 1 };
	double d{ 0.5 };
	double h{ 0.05 };
};

int main()
{
	auto &rs = Aris::ControlServer::Instance();
	rs.CreateModel<Robots::RobotTypeI>();
#ifdef WIN32
	rs.LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	rs.LoadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

	rs.SetParseFunc("en", ParseFunc);
	rs.SetParseFunc("ds", ParseFunc);
	rs.SetParseFunc("hm", ParseFunc);
	
	

	//rs.AddGait("wk", Robots::walk, Robots::parseWalk);
	//rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	//rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
	//rs->AddGait("ro", Robots::resetOrigin, Robots::parseResetOrigin);
	rs.Start();
	std::cout<<"started"<<std::endl;

	
	
	Aris::Core::RunMsgLoop();

	return 0;
}
