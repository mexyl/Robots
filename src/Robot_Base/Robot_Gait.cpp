#include <Platform.h>

#include <cstring>
#include <cmath>
#include <Aris_DynKer.h>
#include <Aris_Plan.h>

#include "Robot_Gait.h"



using namespace Aris::Plan;

namespace Robots
{
	int find_axis(const char *direction, unsigned &axis, int &sign)
	{
		if (!(strcmp(direction, "x")
			&& strcmp(direction, "+x")
			&& strcmp(direction, "X")
			&& strcmp(direction, "+X")))
		{
			axis = 0;
			sign = 1;

			return 0;
		}
		else if (!(strcmp(direction, "-x")
			&& strcmp(direction, "-X")))
		{
			axis = 0;
			sign = -1;

			return 0;
		}
		else if (!(strcmp(direction, "y")
			&& strcmp(direction, "+y")
			&& strcmp(direction, "Y")
			&& strcmp(direction, "+Y")))
		{
			axis = 1;
			sign = 1;

			return 0;
		}
		else if (!(strcmp(direction, "-y")
			&& strcmp(direction, "-Y")))
		{
			axis = 1;
			sign = -1;

			return 0;
		}
		else if (!(strcmp(direction, "z")
			&& strcmp(direction, "+z")
			&& strcmp(direction, "Z")
			&& strcmp(direction, "+Z")))
		{
			axis = 2;
			sign = 1;

			return 0;
		}
		else if (!(strcmp(direction, "-x")
			&& strcmp(direction, "-X")))
		{
			axis = 2;
			sign = -1;

			return 0;
		}
		else
		{
			return -1;
		}
	}


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
		double *pBodyEp)
	{
		memset(pBodyEp, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		unsigned walk_axis, up_axis;
		int walk_sign, up_sign;
		find_axis(walkDirection, walk_axis, walk_sign);
		find_axis(upDirection, up_axis, up_sign);


		double alpha = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

		/*设置移动腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + walk_axis] += walk_sign*(-(d / 4)*cos(alpha) + d / 4);
			pEE[i + up_axis] += up_sign*h*sin(alpha);
		}

		/*设置身体*/
		pBodyEp[3 + walk_axis] = walk_sign*(acc_even(totalCount, (count + 1)) *d / 4);

		pRobot->SetPee(pEE, pBodyEp);
		pRobot->GetPin(pIn);

		return totalCount - count - 1;
	}
	
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
		double *pBodyEp)
	{
		memset(pBodyEp, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		unsigned walk_axis, up_axis;
		int walk_sign, up_sign;
		find_axis(walkDirection, walk_axis, walk_sign);
		find_axis(upDirection, up_axis, up_sign);

		/*设置支撑腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + walk_axis] += walk_sign*d / 2;
		}

		double alpha = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

		/*设置支撑腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			pEE[i + walk_axis] += walk_sign*(-(d / 4)*cos(alpha) + d / 4);
			pEE[i + up_axis] += up_sign*h*sin(alpha);
		}

		/*设置身体*/
		pBodyEp[3 + walk_axis] = walk_sign*(dec_even(totalCount, (count + 1)) *d / 4 + d / 4);

		pRobot->SetPee(pEE, pBodyEp);
		pRobot->GetPin(pIn);

		return totalCount - count - 1;
	}
	
	int home2start(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned period1, unsigned period2,
		const double *homePee,
		const double *firstPee,
		const double *startPee,
		double *pIn,
		double *pEE,
		double *pBodyEp)
	{
		memset(pBodyEp, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, homePee, sizeof(double) * 18);

		if (count < period1)
		{
			double alpha = -(PI / 2)*cos(PI * (count + 1) / period1) + PI / 2;

			for (unsigned i = 0; i < 18; ++i)
			{
				pEE[i] = homePee[i] * (cos(alpha) + 1) / 2 - firstPee[i] * (cos(alpha) - 1) / 2;
			}
		}
		else
		{
			double alpha = -(PI / 2)*cos(PI * (count + 1 - period1) / period2) + PI / 2;//0 to PI,cos(alpha)is 1 to -1

			for (unsigned i = 0; i < 18; ++i)
			{
				pEE[i] = firstPee[i] * (cos(alpha) + 1) / 2 - startPee[i] * (cos(alpha) - 1) / 2;
			}
		}

		pRobot->SetPee(pEE, pBodyEp);
		pRobot->GetPin(pIn);

		return period1 + period2 - count - 1;
	}
}