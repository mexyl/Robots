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
		std::int32_t cmdType{0};
		std::int32_t cmdID{0};
		std::int32_t count{0};
		std::int16_t motorNum{18};
		std::int16_t legNum{6};
		std::int32_t motorID[18]{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
		std::int32_t legID[6]{0,1,2,3,4,5};
		double beginPee[18]{0};
		double beginVee[18]{0};
		double beginBodyPE[6]{0};
		double beginBodyVel[6]{0};
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
		enum { MAX_PERIOD_NUM = 10};
		
		double targetPee[MAX_PERIOD_NUM][18];
		double targetBodyPE[MAX_PERIOD_NUM][6];
		std::int32_t periodCount[MAX_PERIOD_NUM];
		std::int32_t periodNum;
	};

	int walk(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);

	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
}

#endif
