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

void BasicParseFunc(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out) 
{
	Aris::Server::BasicFunctionParam param;
	
	for (auto &i : params)
	{
		if (i.first == "all")
		{
			std::fill_n(param.active_motor, 18, true);
		}
		else if (i.first == "first")
		{
			std::fill_n(param.active_motor, 18, false);
			std::fill_n(param.active_motor + 0, 3, true);
			std::fill_n(param.active_motor + 6, 3, true);
			std::fill_n(param.active_motor + 12, 3, true);
		}
		else if (i.first == "second")
		{
			std::fill_n(param.active_motor, 18, false);
			std::fill_n(param.active_motor + 3, 3, true);
			std::fill_n(param.active_motor + 9, 3, true);
			std::fill_n(param.active_motor + 15, 3, true);
		}
		else if (i.first == "motor")
		{
			std::fill_n(param.active_motor, 18, false);
			int id = { stoi(i.second) };
			param.active_motor[id] = true;
		}
	}

	msg_out.copyStruct(param);
}

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

struct RecoverParam final :public Aris::Server::GaitParamBase
{
	std::int32_t recover_count{ 3000 };
	std::int32_t align_count{ 3000 };
	bool active_leg[6]{ true,true,true,true,true,true };
	double alignPee[18]
	{-0.3,   -0.75,   -0.65,
	-0.45,  -0.75,   0,
	-0.3,   -0.75,    0.65,
	0.3,   -0.75,    -0.65,
	0.45,  -0.75,    0,
	0.3,   -0.75,     0.65};
	double recoverPee[18]
	{ -0.3,   -0.85,   -0.65,
		-0.45,  -0.85,   0,
		-0.3,   -0.85,    0.65,
		0.3,   -0.85,    -0.65,
		0.45,  -0.85,    0,
		0.3,   -0.85,     0.65 };
};
void ParseRecover(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)
{
	RecoverParam param;
	msg_out.copyStruct(param);
}
int Recover(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)
{
	auto &robot = static_cast<Robots::RobotBase &>(model);
	auto &param = static_cast<const RecoverParam &>(plan_param);
	
	static double beginPin[18];
	if (param.count == 0)std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
	
	const double pe[6]{ 0 };
	robot.SetPeb(pe);
	robot.SetPee(param.alignPee);
	double alignPin[18]{ 0 };
	robot.GetPin(alignPin);

	int leftCount = param.count < param.align_count ? 0 : param.align_count;
	int rightCount = param.count < param.align_count ? param.align_count : param.align_count + param.recover_count;

	double s = -(PI / 2)*cos(PI * (param.count - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

	for (int i = 0; i < 6; ++i)
	{
		if (param.active_leg[i])
		{
			if (param.count < param.align_count)
			{
				for (int j = 0; j < 3; ++j)
				{
					robot.motionPool().at(i * 3 + j).setMotPos(beginPin[i * 3 + j] * (cos(s) + 1) / 2 + alignPin[i * 3 + j] * (1 - cos(s)) / 2);
				}
			}
			else
			{
				double pEE[3];
				for (int j = 0; j < 3; ++j)
				{
					pEE[j] = param.alignPee[i * 3 + j] * (cos(s) + 1) / 2 + param.recoverPee[i * 3 + j] * (1 - cos(s)) / 2;
				}

				robot.pLegs[i]->SetPee(pEE);
			}
		}
	}

	return param.align_count + param.recover_count - param.count - 1;
}

int main()
{
	auto &rs = Aris::Server::ControlServer::instance();
	
	rs.createModel<Robots::RobotTypeI>();
#ifdef WIN32
	rs.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	rs.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

	rs.addCmd("en", BasicParseFunc, nullptr);
	rs.addCmd("ds", BasicParseFunc, nullptr);
	rs.addCmd("hm", BasicParseFunc, nullptr);
	rs.addCmd("rc", ParseRecover, Recover);
	rs.addCmd("sw", ParseSimpleWalk, SimpleWalk);
	//rs.addCmd("wk", Robots::parseWalk, Robots::walk);

	rs.open();

	rs.setOnExit([]() {Aris::Core::stopMsgLoop(); });
	Aris::Core::runMsgLoop();

	

	return 0;
}
