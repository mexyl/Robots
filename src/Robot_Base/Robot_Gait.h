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
	};

	int walkAcc(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walkConst(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	int walkDec(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);

	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam);
	//int move(ROBOT_BASE *pRobot, GAIT_PARAM_BASE *pParam, unsigned count);
	
	
	
	
	
	
	int walk_acc(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned totalCount,
		const double *iniPee,
		double d, 
		double h,
		double alpha,//平面内直线行走相对于walkAxis绕upAxis转动的角度
		double beta,//一个const周期后，机器人相对于upAxis转动的角度
		const char *walkDirection,
		const char *upDirection,
		double *pIn,
		double *pEE,
		double *pBodyEp);

	int walk_dec(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned totalCount,
		const double *iniPee,
		double d, 
		double h,
		double alpha,//平面内直线行走相对于walkAxis的角度
		double beta,//一个const周期后，机器人相对于upAxis转动的角度
		const char *walkAxis,
		const char *upAxis,
		double *pIn,
		double *pEE,
		double *pBodyEp);

	int walk_const(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned totalCount,
		const double *iniPee,
		double d, 
		double h,
		double alpha,//平面内直线行走相对于walkAxis的角度
		double beta,//一个const周期后，机器人相对于upAxis转动的角度
		const char *walkAxis,
		const char *upAxis,
		double *pIn,
		double *pEE,
		double *pBodyEp);

	int move(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned totalCount,
		const double *beginPee,
		const double *beginBodyEp,
		const double *beginBodyVel,
		const double *endPee,
		const double *endBodyEp,
		const double *endBodyVel,
		double h,
		int upAxis,
		double *pIn,
		double *pEE,
		double *pBodyEp);

	int home2start(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned period1, unsigned period2,
		const double *homePee,
		const double *firstPee,
		const double *startPee,
		double *pIn,
		double *pEE,
		double *pBodyEp);


	




}

#endif
