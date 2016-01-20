#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris_core.h>
#include <aris_motion.h>
#include <aris_imu.h>
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

	/*for enable, disable, and home*/
	struct BasicFunctionParam final :AllParamBase
	{
		bool isMotorActive[18]{ true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true };
	};

	/*for recover*/
	struct RecoverParam final :AllParamBase
	{
		std::int32_t alignCount{ 3000 };
		std::int32_t recoverCount{ 3000 };
		double beginPin[18]{ 0 };
		double alignPin[18]{ 0 };
		double alignPee[18]{ 0 };
		double recoverPee[18]{ 0 };
		bool isLegActive[6]{ true,true,true,true,true,true };
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

	struct WalkParam final:public GaitParamBase
	{
		std::int32_t totalCount{3000};
		std::int32_t n{1};
		std::int32_t walkDirection{-3};// 1 means positive x axis; while -3 means negative z axis
		std::int32_t upDirection{2};
		double d{0.5};
		double h{0.05};
		double alpha{0};
		double beta{0};
	};
	int walk(RobotBase * pRobot, const GaitParamBase * pParam);
	Aris::Core::Msg parseWalk(const std::string &cmd, const std::map<std::string, std::string> &params);
	
	struct AdjustParam :public GaitParamBase
	{
		enum { MAX_PERIOD_NUM = 10};
		
		std::int16_t motorNum{ 18 };
		std::int16_t legNum{ 6 };
		std::int32_t motorID[18]{ 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 };
		std::int32_t legID[6]{ 0,1,2,3,4,5 };
		double targetPee[MAX_PERIOD_NUM][18];
		double targetPeb[MAX_PERIOD_NUM][6];
		std::int32_t periodCount[MAX_PERIOD_NUM]{1000};
		std::int32_t periodNum{1};
		char relativeCoordinate[8]{ 'G',0 };
		char relativeBodyCoordinate[8]{ 'G',0 };
	};
	int adjust(RobotBase * pRobot, const GaitParamBase * pParam);
	Aris::Core::Msg parseAdjust(const std::string &cmd, const std::map<std::string, std::string> &params);

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
