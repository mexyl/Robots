#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <Robot_Base.h>

namespace Robots
{
	auto basicParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void;
	
	struct FakeHomeParam final :public Aris::Server::GaitParamBase {};
	auto fakeHomeParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void;
	auto fakeHomeGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)->int;

	struct RecoverParam final :public Aris::Server::GaitParamBase
	{
		std::int32_t recover_count{ 3000 };
		std::int32_t align_count{ 3000 };
		bool active_leg[6]{ true,true,true,true,true,true };
		double alignPee[18]
		{ -0.3,   -0.75,   -0.65,
			-0.45,  -0.75,   0,
			-0.3,   -0.75,    0.65,
			0.3,   -0.75,    -0.65,
			0.45,  -0.75,    0,
			0.3,   -0.75,     0.65 };
		double recoverPee[18]
		{ -0.3,   -0.85,   -0.65,
			-0.45,  -0.85,   0,
			-0.3,   -0.85,    0.65,
			0.3,   -0.85,    -0.65,
			0.45,  -0.85,    0,
			0.3,   -0.85,     0.65 };
	};
	auto recoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void;
	auto recoverGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)->int;
	
	struct WalkParam final:public Aris::Server::GaitParamBase
	{
		std::int32_t totalCount{ 3000 };
		std::int32_t n{ 2 };
		double d{ 0.5 };
		double h{ 0.05 };
		double alpha{ 0.3 };
		double beta{ 0.3 };
	};
	auto walkParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)->void;
	auto walkGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param)->int;
}

#endif
