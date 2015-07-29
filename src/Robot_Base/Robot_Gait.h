#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include <functional>
#include <cstdint>

#include <Robot_Base.h>

namespace Robots
{
	class ROBOT_BASE;
	
	struct GAIT_PARAM_BASE
	{
		std::int32_t cmdType;
		std::int32_t cmdID;
		std::int32_t count;
		std::int16_t motorNum;
		std::int16_t legNum;
		std::int32_t motorID[18];
		std::int32_t legID[6];
		double beginPee[18];
		double beginVee[18];
		double beginBodyPE[6];
		double beginBodyVel[6];
	};

	struct WALK_PARAM :public GAIT_PARAM_BASE
	{
		std::int32_t totalCount;
		std::int32_t n;
		std::int32_t walkDirection;// 1 means positive x axis; while -3 means negative z axis
		std::int32_t upDirection;
		double d;
		double h;
		double alpha;
		double beta;
	};
	struct ADJUST_PARAM :public GAIT_PARAM_BASE
	{
		double targetPee[18];
		double targetBodyPE[6];
		std::int32_t totalCount;
	};

	int walkAcc(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walkConst(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walkDec(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);

	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
}

#endif
