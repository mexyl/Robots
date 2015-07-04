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
		double d, double h,
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
		double d, double h,
		const char *walkDirection,
		const char *upDirection,
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