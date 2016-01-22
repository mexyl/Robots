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
#include <aris_motion.h>
#include <Robot_Server.h>
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
	Aris::BasicFunctionParam param;
	
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

	msg_out.CopyStruct(param);
}

struct SimpleWalkParam final :public Aris::Dynamic::PlanParamBase
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
int SimpleWalk(Aris::Dynamic::ModelBase &model, const Aris::Dynamic::PlanParamBase & param)
{
	auto &sp = static_cast<const SimpleWalkParam&>(param);
	auto &robot = static_cast<Robots::RobotTypeI&>(model);

	static Aris::Dynamic::FloatMarker beginBodyMak(robot.Ground(), nullptr, "313");
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

struct RecoverParam final :public Aris::GaitParamBase
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
	msg_out.CopyStruct(param);
}
int Recover(Aris::Dynamic::ModelBase &model, const Aris::Dynamic::PlanParamBase & plan_param)
{
	auto &robot = static_cast<Robots::RobotBase &>(model);
	auto &param = static_cast<const RecoverParam &>(plan_param);
	
	//写入初值
	static double beginPin[18];

	if (param.count == 0)std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);

	/*
	if (param.count % 100)
	{
		for (int i = 0; i<18; ++i)
		{
			rt_printf("%f ", beginPin[i]);
		}
		rt_printf("\n");
	}*/

	
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
					robot.MotionAt(i * 3 + j).SetMotPos(beginPin[i * 3 + j] * (cos(s) + 1) / 2 + alignPin[i * 3 + j] * (1 - cos(s)) / 2);
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
	
	//向下写入输入位置
	return param.align_count + param.recover_count - param.count - 1;
}

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

	rs.SetParseFunc("en", BasicParseFunc);
	rs.SetParseFunc("ds", BasicParseFunc);
	rs.SetParseFunc("hm", BasicParseFunc);
	rs.AddGait("rc", Recover, ParseRecover);
	

	//rs.AddGait("wk", Robots::walk, Robots::parseWalk);
	//rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	//rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
	//rs->AddGait("ro", Robots::resetOrigin, Robots::parseResetOrigin);
	rs.Start();
	std::cout<<"started"<<std::endl;

	
	
	Aris::Core::RunMsgLoop();

	return 0;
}
