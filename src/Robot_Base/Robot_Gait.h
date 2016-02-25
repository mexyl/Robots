#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <Robot_Base.h>

namespace Robots
{
	class RobotBase;
	struct GaitParamBase;
	typedef std::function<int(RobotBase *, const GaitParamBase *)> GaitFunc;

	struct AllParamBase
	{
		std::int32_t cmdType{ 0 };
		std::int32_t cmdID{ 0 };
		mutable std::int32_t count{ 0 };
	};

	struct GaitParamBase:AllParamBase
	{
		const Aris::Sensor::ImuData *imuData;
		const std::vector<Aris::Control::EthercatForceSensor::Data> *pForceData;
		double beginPee[18]{0};
		double beginVee[18]{0};
		double beginPeb[6]{0};
		double beginVb[6]{0};
	};

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
	Aris::Core::Msg parseWalk(const std::string &cmd, const std::map<std::string, std::string> &params);
	
	struct FastWalkParam :public GaitParamBase
	{
		const char fileName[256]{ 0 };
		std::int32_t accCount{ 0 };
		std::int32_t decCount{ 0 };
		std::int32_t constCount{ 0 };
		std::int32_t n{ 1 };
		double *pInAcc{ nullptr };
		double *pInDec{ nullptr };
		double *pInConst{ nullptr };
	};
	int fastWalk(RobotBase * pRobot, const GaitParamBase * pParam);
	Aris::Core::Msg parseFastWalk(const std::string &cmd, const std::map<std::string, std::string> &params);

	int resetOrigin(RobotBase * pRobot, const Robots::GaitParamBase *pParam);
	Aris::Core::Msg parseResetOrigin(const std::string &cmd, const std::map<std::string, std::string> &params);
}

#endif
