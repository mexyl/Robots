#include <Platform.h>

#include <cstring>
#include <cmath>
#include <Aris_DynKer.h>
#include <Aris_Plan.h>

#include "Robot_Gait.h"



using namespace Aris::Plan;
using namespace Aris::DynKer;

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
		double d, 
		double h, 
		double alpha,//平面内直线行走相对于walkAxis的角度
		double beta,//一个const周期后，机器人相对于upAxis转动的角度
		const char *walkAxis,
		const char *upAxis,
		double *pIn, 
		double *pEE, 
		double *pBodyEp)
	{
		memset(pBodyEp, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		/*求解三个坐标轴及其符号*/
		unsigned walk_axis, up_axis, side_axis;
		int walk_sign, up_sign, side_sign;
		find_axis(walkAxis, walk_axis, walk_sign);
		find_axis(upAxis, up_axis, up_sign);
		side_axis = 3 - up_axis - walk_axis;
		if (((3 + walk_axis - up_axis) % 3) == 1)
		{
			side_sign = walk_sign* up_sign;
		}
		else
		{
			side_sign = -walk_sign* up_sign;
		}

		/*求解当前在地面坐标系下的初始位置*/
		double beginPos[18];
		memcpy(beginPos, iniPee, sizeof(double) * 18);

		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

		/*设置移动腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + walk_axis] = walk_sign*0.5*d / cos(beta / 2)*cos(beta / 4)*(1 - cos(s)) / 2
				+ beginPos[i + walk_axis] * cos((1 - cos(s)) / 4 * beta)
				- beginPos[i + side_axis] * sin((1 - cos(s)) / 4 * beta);
			pEE[i + up_axis] = up_sign*h*sin(s)
				+ beginPos[i + up_axis];
			pEE[i + side_axis] = -side_sign*0.5*d / cos(beta / 2)*sin(beta / 4)*(1 - cos(s)) / 2
				+ beginPos[i + walk_axis] * sin((1 - cos(s)) / 4 * beta)
				+ beginPos[i + side_axis] * cos((1 - cos(s)) / 4 * beta);
		}

		/*设置支撑腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			pEE[i + walk_axis] = beginPos[i + walk_axis];
			pEE[i + up_axis] = beginPos[i + up_axis];
			pEE[i + side_axis] = beginPos[i + side_axis];
		}


		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		double ap[6] = { 0 }, ep[6];
		double pm[4][4];
		ap[up_axis] = up_sign*beta / 4 * acc_even(totalCount,count + 1);
		s_ap2pm(ap, *pm);
		s_pm2ep(*pm, ep);

		memcpy(pBodyEp, ep, sizeof(double) * 6);

		pBodyEp[3 + walk_axis] = walk_sign*0.25*d / cos(beta / 2)*cos(beta / 4)*(acc_even(totalCount, count + 1));
		pBodyEp[3 + up_axis] = 0;
		pBodyEp[3 + side_axis] = -side_sign*0.25*d / cos(beta / 2)*sin(beta / 4)*(acc_even(totalCount, count + 1));


		pRobot->SetPee(pEE,pBodyEp);
		//pRobot->GetPee(pEE, "B");
		pRobot->GetPin(pIn);

		return totalCount - count - 1;
	}
	
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
		double *pBodyEp)
	{
		

		memset(pBodyEp, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		/*求解三个坐标轴及其符号*/
		unsigned walk_axis, up_axis, side_axis;
		int walk_sign, up_sign, side_sign;
		find_axis(walkAxis, walk_axis, walk_sign);
		find_axis(upAxis, up_axis, up_sign);
		side_axis = 3 - up_axis - walk_axis;
		if (((3 + walk_axis - up_axis) % 3) == 1)
		{
			side_sign = walk_sign* up_sign;
		}
		else
		{
			side_sign = -walk_sign* up_sign;
		}

		

		/*求解当前在地面坐标系下的初始位置*/
		double beginPos[18];
		memcpy(beginPos, iniPee, sizeof(beginPos));
		for (unsigned i = 0; i < 18; i += 6)
		{
			beginPos[i + walk_axis] = walk_sign*0.5*d / cos(beta / 2)*cos(beta / 4)
				+ iniPee[i + walk_axis] * cos(beta/2)
				- iniPee[i + side_axis] * sin(beta/2);
			beginPos[i + up_axis] =
				+ iniPee[i + up_axis];
			beginPos[i + side_axis] = -side_sign*0.5*d / cos(beta / 2)*sin(beta / 4)
				+ iniPee[i + walk_axis] * sin(beta/2)
				+ iniPee[i + side_axis] * cos(beta/2);
		}


		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

		/*设置移动腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			pEE[i + walk_axis] = walk_sign*0.5*d / cos(beta / 2)*cos(beta / 4)*(1 - cos(s)) / 2
				+ beginPos[i + walk_axis] * cos((1 - cos(s)) / 4 * beta)
				- beginPos[i + side_axis] * sin((1 - cos(s)) / 4 * beta);
			pEE[i + up_axis] = up_sign*h*sin(s)
				+ beginPos[i + up_axis];
			pEE[i + side_axis] = -side_sign*0.5*d / cos(beta / 2)*sin(beta / 4)*(1 - cos(s)) / 2
				+ beginPos[i + walk_axis] * sin((1 - cos(s)) / 4 * beta)
				+ beginPos[i + side_axis] * cos((1 - cos(s)) / 4 * beta);
		}

		/*设置支撑腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + walk_axis] = beginPos[i + walk_axis];
			pEE[i + up_axis] = beginPos[i + up_axis];
			pEE[i + side_axis] = beginPos[i + side_axis];
		}

		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		double ap[6] = { 0 }, ep[6];
		double pm[4][4];
		ap[up_axis] = up_sign*(beta / 4 * (dec_even(totalCount, count + 1) + 1));
		s_ap2pm(ap, *pm);
		s_pm2ep(*pm, ep);

		memcpy(pBodyEp, ep, sizeof(double) * 6);

		pBodyEp[3 + walk_axis] = walk_sign*0.25*d / cos(beta / 2)*cos(beta / 4)*(dec_even(totalCount, count + 1) + 1);
		pBodyEp[3 + up_axis] = 0;
		pBodyEp[3 + side_axis] = -side_sign*0.25*d / cos(beta / 2)*sin(beta / 4)*(dec_even(totalCount, count + 1) + 1);











		/*向外复制*/
		pRobot->SetPee(pEE, pBodyEp);
		pRobot->GetPin(pIn);

		return totalCount - count - 1;
	}

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
		double *pBodyEp)
	{
		
		
		/*此时totalCount为半个周期，它应等于acc中的totalCount*/
		memset(pBodyEp, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		/*求解三个坐标轴及其符号*/
		unsigned walk_axis, up_axis, side_axis;
		int walk_sign, up_sign, side_sign;
		find_axis(walkAxis, walk_axis, walk_sign);
		find_axis(upAxis, up_axis, up_sign);
		side_axis = 3 - up_axis - walk_axis;
		if (((3 + walk_axis - up_axis) % 3) == 1)
		{
			side_sign = walk_sign* up_sign;
		}
		else
		{
			side_sign = -walk_sign* up_sign;
		}

		/*求解当前在地面坐标系下的初始位置*/
		double beginPos[18];
		memcpy(beginPos, iniPee, sizeof(beginPos));
		/*这是上一轮的移动腿，也就是马上支撑的腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			beginPos[i + walk_axis] = walk_sign*0.25*d / cos(beta / 2)*cos(beta / 2)
				+ iniPee[i + walk_axis] * cos(beta / 4)
				- iniPee[i + side_axis] * sin(beta / 4);
			beginPos[i + up_axis] =
				+ iniPee[i + up_axis];
			beginPos[i + side_axis] = -side_sign*0.25*d / cos(beta / 2)*sin(beta / 2)
				+ iniPee[i + walk_axis] * sin(beta / 4)
				+ iniPee[i + side_axis] * cos(beta / 4);
		}
		/*这是上一轮的支撑腿，也就是马上移动的腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			beginPos[i + walk_axis] = -walk_sign*0.25*d / cos(beta / 2)*cos(beta / 2)
				+ iniPee[i + walk_axis] * cos(beta / 4)
				+ iniPee[i + side_axis] * sin(beta / 4);
			beginPos[i + up_axis] =
				+iniPee[i + up_axis];
			beginPos[i + side_axis] = side_sign*0.25*d / cos(beta / 2)*sin(beta / 2)
				- iniPee[i + walk_axis] * sin(beta / 4)
				+ iniPee[i + side_axis] * cos(beta / 4);
		}

		if (count < totalCount)
		{
			/*此时在前半周期*/

			/*设置支撑腿*/
			for (unsigned i = 0; i < 18; i += 6)
			{
				pEE[i + walk_axis] = beginPos[i + walk_axis];
				pEE[i + up_axis] = beginPos[i + up_axis];
				pEE[i + side_axis] = beginPos[i + side_axis];
			}

			double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

			/*设置移动腿*/
			for (unsigned i = 3; i < 18; i += 6)
			{
				pEE[i + walk_axis] = walk_sign*(-(d / 2)*cos(s) + d / 2)
					+ beginPos[i + walk_axis] * cos((1 - cos(s)) / 2 * beta)
					- beginPos[i + side_axis] * sin((1 - cos(s)) / 2 * beta);
				pEE[i + up_axis] = up_sign*h*sin(s)
					+ beginPos[i + up_axis];
				pEE[i + side_axis] = 
					+ beginPos[i + walk_axis] * sin((1 - cos(s)) / 2 * beta)
					+ beginPos[i + side_axis] * cos((1 - cos(s)) / 2 * beta);
			}
		}
		else
		{
			/*设置支撑腿*/
			for (unsigned i = 3; i < 18; i += 6)
			{
				pEE[i + walk_axis] = walk_sign * d
					+ beginPos[i + walk_axis] * cos(beta)
					- beginPos[i + side_axis] * sin(beta);
				pEE[i + up_axis] =
					+ beginPos[i + up_axis];
				pEE[i + side_axis] = 
					+ beginPos[i + walk_axis] * sin(beta)
					+ beginPos[i + side_axis] * cos(beta);
			}

			double s = -(PI / 2)*cos(PI * (count + 1 - totalCount) / totalCount) + PI / 2;

			/*设置移动腿*/
			for (unsigned i = 0; i < 18; i += 6)
			{
				pEE[i + walk_axis] = walk_sign*(-(d / 2)*cos(s) + d / 2)
					+ beginPos[i + walk_axis] * cos((1 - cos(s)) / 2 * beta)
					- beginPos[i + side_axis] * sin((1 - cos(s)) / 2 * beta);
				pEE[i + up_axis] = up_sign*h*sin(s)
					+ beginPos[i + up_axis];
				pEE[i + side_axis] =
					+beginPos[i + walk_axis] * sin((1 - cos(s)) / 2 * beta)
					+ beginPos[i + side_axis] * cos((1 - cos(s)) / 2 * beta);
			}
		}
		
		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		double ap[6] = { 0 }, ep[6];
		double pm[4][4];
		ap[up_axis] = up_sign*beta*(count+1)/totalCount/2;
		s_ap2pm(ap, *pm);
		s_pm2ep(*pm, ep);

		memcpy(pBodyEp, ep, sizeof(double) * 6);

		pBodyEp[3 + walk_axis] = walk_sign*d*(t / T);
		pBodyEp[3 + up_axis] = 0;
		pBodyEp[3 + side_axis] = -side_sign*(tan(beta / 2)*d*t / T*(1 - t / T));

		






		pRobot->SetPee(pEE, pBodyEp);
		pRobot->GetPin(pIn);

		return 2 * totalCount - count - 1;
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