#ifndef ROBOT_GAIT_H
#define ROBOT_GAIT_H

#include "Robot_Base.h"

namespace Robots
{
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