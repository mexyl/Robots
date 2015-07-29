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
		std::int32_t cmdID;			
		std::uint32_t count;	
		double beginPee[18];
		double beginVee[18];
		double beginBodyPE[6];
		double beginBodyVel[6];
	};

	struct WALK_PARAM :public GAIT_PARAM_BASE
	{
		std::uint32_t totalCount;
		std::uint32_t n;
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
		std::uint32_t totalCount;
		std::uint32_t motorNum;
		std::uint32_t motorID[18];
	};

	int walkAcc(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walkConst(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walkDec(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);

	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
}

#endif
