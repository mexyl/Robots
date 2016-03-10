#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <Robot_Base.h>

namespace Robots
{
	struct WalkParam final:public Aris::Server::GaitParamBase
	{
		std::int32_t totalCount{3000};
		std::int32_t n{2};
		double d{0.5};
		double h{0.05};
		double alpha{0};
		double beta{0.3};
	};
	int walk(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param);
	void parseWalk(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg);
}

#endif
