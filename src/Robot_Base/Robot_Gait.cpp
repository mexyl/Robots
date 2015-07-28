#include <Platform.h>

#include <cstring>
#include <cmath>
#include <Aris_DynKer.h>
#include <Aris_Plan.h>

#include "Robot_Gait.h"
#include "Robot_Base.h"


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
		else if (!(strcmp(direction, "-z")
			&& strcmp(direction, "-Z")))
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

	int walkAcc(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		/*初始化参数*/
		const WALK_PARAM *pRealParam = static_cast<const WALK_PARAM *>(pParam);

		unsigned wAxis = std::abs(pRealParam->walkDirection) - 1;
		unsigned uAxis = std::abs(pRealParam->upDirection) - 1;
		unsigned lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginBodyPE, *pm, "313");
		s_pm2pe(*pm, pe, "321");

		unsigned totalCount = pRealParam->totalCount;
		double h = pRealParam->h;
		double d = pRealParam->d;
		double a = pRealParam->alpha + uSign*pe[5 - uAxis];
		double b = pRealParam->beta;

		const double *beginPee = pRealParam->beginPee;
		const double *beginBodyPE = pRealParam->beginBodyPE;

		double pEE[18];
		double pBodyPE[6];

		/*初始化完毕，开始计算*/
		unsigned count = pRealParam->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
		/*设置移动腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a - b / 4)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos((1 - cos(s)) / 4 * b)
				- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin((1 - cos(s)) / 4 * b))
				+ beginBodyPE[wAxis];
			pEE[i + uAxis] = uSign*h*sin(s)
				+ beginPee[i + uAxis];
			pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a - b / 4)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin((1 - cos(s)) / 4 * b)
				+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos((1 - cos(s)) / 4 * b))
				+ beginBodyPE[lAxis];
		}
		/*设置支撑腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			pEE[i + wAxis] = beginPee[i + wAxis];
			pEE[i + uAxis] = beginPee[i + uAxis];
			pEE[i + lAxis] = beginPee[i + lAxis];
		}

		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		/*以下计算角度，需要在313和321的欧拉角中间转来转去*/
		double beginPE321[6];
		s_pe2pm(pParam->beginBodyPE, *pm);
		s_pm2pe(*pm, beginPE321, "321");
		beginPE321[5 - uAxis] += uSign*b / 4 * acc_even(totalCount, count + 1);

		s_pe2pm(beginPE321, *pm, "321");
		s_pm2pe(*pm, pBodyPE);

		/*以下计算位置*/
		pBodyPE[wAxis] += wSign*0.25*d / cos(b / 2)*cos(a - b / 4)*(acc_even(totalCount, count + 1));
		pBodyPE[uAxis] += 0;
		pBodyPE[lAxis] += lSign*0.25*d / cos(b / 2)*sin(a - b / 4)*(acc_even(totalCount, count + 1));

		/*计算完毕，更新pRobot*/
		pRobot->SetPee(pEE, pBodyPE, "G");
		return totalCount - count - 1;
	}
	int walkDec(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		/*初始化参数*/
		const WALK_PARAM *pRealParam = static_cast<const WALK_PARAM *>(pParam);

		unsigned wAxis = std::abs(pRealParam->walkDirection) - 1;
		unsigned uAxis = std::abs(pRealParam->upDirection) - 1;
		unsigned lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginBodyPE, *pm, "313");
		s_pm2pe(*pm, pe, "321");

		unsigned totalCount = pRealParam->totalCount;
		double h = pRealParam->h;
		double d = pRealParam->d;
		double a = pRealParam->alpha + uSign*pe[5 - uAxis];
		double b = pRealParam->beta;

		const double *beginPee = pRealParam->beginPee;
		const double *beginBodyPE = pRealParam->beginBodyPE;

		double pEE[18];
		double pBodyPE[6];

		/*初始化完毕，开始计算*/
		unsigned count = pRealParam->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
		/*设置移动腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			double aaa = cos((1 - cos(s)) / 4 * b);

			pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a - b / 2)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a - b / 2)) * cos((1 - cos(s)) / 4 * b)
				- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a - b / 2)) * sin((1 - cos(s)) / 4 * b))
				+ beginBodyPE[wAxis] - wSign*0.25*d / cos(b / 2)*cos(a - b / 2);
			pEE[i + uAxis] = uSign*h*sin(s)
				+ beginPee[i + uAxis];
			pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a - b / 2)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a - b / 2)) * sin((1 - cos(s)) / 4 * b)
				+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a - b / 2)) * cos((1 - cos(s)) / 4 * b))
				+ beginBodyPE[lAxis] - lSign*0.25*d / cos(b / 2)*sin(a - b / 2);
		}



		/*设置支撑腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + wAxis] = beginPee[i + wAxis];
			pEE[i + uAxis] = beginPee[i + uAxis];
			pEE[i + lAxis] = beginPee[i + lAxis];
		}

		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		/*以下计算角度，需要在313和321的欧拉角中间转来转去*/
		double beginPE321[6];
		s_pe2pm(pParam->beginBodyPE, *pm);
		s_pm2pe(*pm, beginPE321, "321");
		beginPE321[5 - uAxis] += uSign*b / 4 * dec_even(totalCount, count + 1);

		s_pe2pm(beginPE321, *pm, "321");
		s_pm2pe(*pm, pBodyPE);

		/*以下计算位置*/
		pBodyPE[wAxis] += wSign*0.25*d / cos(b / 2)*cos(a - b / 2)*(dec_even(totalCount, count + 1));
		pBodyPE[uAxis] += 0;
		pBodyPE[lAxis] += lSign*0.25*d / cos(b / 2)*sin(a - b / 2)*(dec_even(totalCount, count + 1));



		/*计算完毕，更新pRobot*/
		pRobot->SetPee(pEE, pBodyPE, "G");
		return totalCount - count - 1;
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
		double *pBodyPE)
	{
		memset(pBodyPE, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		/*求解三个坐标轴及其符号*/
		unsigned walk_axis, up_axis, side_axis;
		int walk_sign, up_sign, left_sign;
		find_axis(walkAxis, walk_axis, walk_sign);
		find_axis(upAxis, up_axis, up_sign);
		side_axis = 3 - up_axis - walk_axis;
		if (((3 + walk_axis - up_axis) % 3) == 1)// find left axis sign
		{
			left_sign = walk_sign* up_sign;
		}
		else
		{
			left_sign = -walk_sign* up_sign;
		}

		/*求解当前在地面坐标系下的初始位置*/
		double beginPos[18];
		memcpy(beginPos, iniPee, sizeof(double) * 18);

		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

		/*设置移动腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			pEE[i + walk_axis] = walk_sign*(0.5*d / cos(beta / 2)*cos(alpha - beta / 4)*(1 - cos(s)) / 2
				+ walk_sign*beginPos[i + walk_axis] * cos((1 - cos(s)) / 4 * beta)
				- left_sign*beginPos[i + side_axis] * sin((1 - cos(s)) / 4 * beta));
			pEE[i + up_axis] = up_sign*h*sin(s)
				+ beginPos[i + up_axis];
			pEE[i + side_axis] = left_sign*(0.5*d / cos(beta / 2)*sin(alpha - beta / 4)*(1 - cos(s)) / 2
				+ walk_sign*beginPos[i + walk_axis] * sin((1 - cos(s)) / 4 * beta)
				+ left_sign*beginPos[i + side_axis] * cos((1 - cos(s)) / 4 * beta));
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

		double pr[6] = { 0 }, pe[6] = { 0 };
		double pm[4][4];
		pr[up_axis + 3] = up_sign*beta / 4 * acc_even(totalCount, count + 1);
		s_pr2pm(pr, *pm);
		s_pm2pe(*pm, pe);

		memcpy(pBodyPE, pe, sizeof(double) * 6);

		pBodyPE[walk_axis] = walk_sign*0.25*d / cos(beta / 2)*cos(alpha - beta / 4)*(acc_even(totalCount, count + 1));
		pBodyPE[up_axis] = 0;
		pBodyPE[side_axis] = left_sign*0.25*d / cos(beta / 2)*sin(alpha - beta / 4)*(acc_even(totalCount, count + 1));


		pRobot->SetPee(pEE, pBodyPE);
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
		double *pBodyPE)
	{
		memset(pBodyPE, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		/*求解三个坐标轴及其符号*/
		unsigned walk_axis, up_axis, side_axis;
		int walk_sign, up_sign, left_sign;
		find_axis(walkAxis, walk_axis, walk_sign);
		find_axis(upAxis, up_axis, up_sign);
		side_axis = 3 - up_axis - walk_axis;
		if (((3 + walk_axis - up_axis) % 3) == 1)
		{
			left_sign = walk_sign* up_sign;
		}
		else
		{
			left_sign = -walk_sign* up_sign;
		}

		

		/*求解当前在地面坐标系下的初始位置*/
		double beginPos[18];
		memcpy(beginPos, iniPee, sizeof(beginPos));
		for (unsigned i = 0; i < 18; i += 6)
		{
			beginPos[i + walk_axis] = walk_sign*(0.5*d / cos(beta / 2)*cos(alpha - beta / 4)
				+ walk_sign*iniPee[i + walk_axis] * cos(beta / 2)
				- left_sign*iniPee[i + side_axis] * sin(beta / 2));
			beginPos[i + up_axis] =
				+ iniPee[i + up_axis];
			beginPos[i + side_axis] = left_sign*(0.5*d / cos(beta / 2)*sin(alpha - beta / 4)
				+ walk_sign*iniPee[i + walk_axis] * sin(beta / 2)
				+ left_sign*iniPee[i + side_axis] * cos(beta / 2));
		}


		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

		/*设置移动腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			pEE[i + walk_axis] = walk_sign*(0.5*d / cos(beta / 2)*cos(alpha - beta / 4)*(1 - cos(s)) / 2
				+ walk_sign*beginPos[i + walk_axis] * cos((1 - cos(s)) / 4 * beta)
				- left_sign*beginPos[i + side_axis] * sin((1 - cos(s)) / 4 * beta));
			pEE[i + up_axis] = up_sign*h*sin(s)
				+ beginPos[i + up_axis];
			pEE[i + side_axis] = left_sign*(0.5*d / cos(beta / 2)*sin(alpha - beta / 4)*(1 - cos(s)) / 2
				+ walk_sign*beginPos[i + walk_axis] * sin((1 - cos(s)) / 4 * beta)
				+ left_sign*beginPos[i + side_axis] * cos((1 - cos(s)) / 4 * beta));
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

		double pr[6] = { 0 }, pe[6];
		double pm[4][4];
		pr[up_axis+3] = up_sign*(beta / 4 * (dec_even(totalCount, count + 1) + 1));
		s_pr2pm(pr, *pm);
		s_pm2pe(*pm, pe);

		memcpy(pBodyPE, pe, sizeof(double) * 6);

		pBodyPE[walk_axis] = walk_sign*0.25*d / cos(beta / 2)*cos(alpha - beta / 4)*(dec_even(totalCount, count + 1) + 1);
		pBodyPE[up_axis] = 0;
		pBodyPE[side_axis] = left_sign*0.25*d / cos(beta / 2)*sin(alpha - beta / 4)*(dec_even(totalCount, count + 1) + 1);











		/*向外复制*/
		pRobot->SetPee(pEE, pBodyPE);
		//pRobot->GetPee(pEE, "B");
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
		double *pBodyPE)
	{
		/*此时totalCount为半个周期，它应等于acc中的totalCount*/
		memset(pBodyPE, 0, sizeof(double) * 6);
		memset(pIn, 0, sizeof(double) * 18);
		memcpy(pEE, iniPee, sizeof(double) * 18);

		/*求解三个坐标轴及其符号*/
		unsigned walk_axis, up_axis, side_axis;
		int walk_sign, up_sign, left_sign;
		find_axis(walkAxis, walk_axis, walk_sign);
		find_axis(upAxis, up_axis, up_sign);
		side_axis = 3 - up_axis - walk_axis;
		if (((3 + walk_axis - up_axis) % 3) == 1)
		{
			left_sign = walk_sign* up_sign;
		}
		else
		{
			left_sign = -walk_sign* up_sign;
		}

		/*求解当前在地面坐标系下的初始位置*/
		double beginPos[18];
		memcpy(beginPos, iniPee, sizeof(beginPos));
		/*这是上一轮的移动腿，也就是马上支撑的腿*/
		for (unsigned i = 0; i < 18; i += 6)
		{
			beginPos[i + walk_axis] = walk_sign*(0.25*d / cos(beta / 2)*cos(alpha - beta / 2)
				+ walk_sign*iniPee[i + walk_axis] * cos(beta / 4)
				- left_sign*iniPee[i + side_axis] * sin(beta / 4));
			beginPos[i + up_axis] =
				+ iniPee[i + up_axis];
			beginPos[i + side_axis] = left_sign*(0.25*d / cos(beta / 2)*sin(alpha - beta / 2)
				+ walk_sign*iniPee[i + walk_axis] * sin(beta / 4)
				+ left_sign*iniPee[i + side_axis] * cos(beta / 4));
		}
		/*这是上一轮的支撑腿，也就是马上移动的腿*/
		for (unsigned i = 3; i < 18; i += 6)
		{
			beginPos[i + walk_axis] = walk_sign*(-0.25*d / cos(beta / 2)*cos(alpha - beta / 2)
				+ walk_sign*iniPee[i + walk_axis] * cos(beta / 4)
				+ left_sign*iniPee[i + side_axis] * sin(beta / 4));
			beginPos[i + up_axis] =
				+iniPee[i + up_axis];
			beginPos[i + side_axis] = left_sign*(-0.25*d / cos(beta / 2)*sin(alpha - beta / 2)
				- walk_sign*iniPee[i + walk_axis] * sin(beta / 4)
				+ left_sign*iniPee[i + side_axis] * cos(beta / 4));
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
				pEE[i + walk_axis] = walk_sign*((-(d / 2)*cos(s) + d / 2) * cos(alpha)
					+ walk_sign*beginPos[i + walk_axis] * cos((1 - cos(s)) / 2 * beta)
					- left_sign*beginPos[i + side_axis] * sin((1 - cos(s)) / 2 * beta));
				pEE[i + up_axis] = up_sign*h*sin(s)
					+ beginPos[i + up_axis];
				pEE[i + side_axis] = left_sign*((-(d / 2)*cos(s) + d / 2) * sin(alpha)
					+ walk_sign*beginPos[i + walk_axis] * sin((1 - cos(s)) / 2 * beta)
					+ left_sign*beginPos[i + side_axis] * cos((1 - cos(s)) / 2 * beta));
			}
		}
		else
		{
			/*设置支撑腿*/
			for (unsigned i = 3; i < 18; i += 6)
			{
				pEE[i + walk_axis] = walk_sign * (d * cos(alpha)
					+ walk_sign*beginPos[i + walk_axis] * cos(beta)
					- left_sign*beginPos[i + side_axis] * sin(beta));
				pEE[i + up_axis] =
					+ beginPos[i + up_axis];
				pEE[i + side_axis] = left_sign * (d * sin(alpha)
					+ walk_sign*beginPos[i + walk_axis] * sin(beta)
					+ left_sign*beginPos[i + side_axis] * cos(beta));
			}

			double s = -(PI / 2)*cos(PI * (count + 1 - totalCount) / totalCount) + PI / 2;

			/*设置移动腿*/
			for (unsigned i = 0; i < 18; i += 6)
			{
				pEE[i + walk_axis] = walk_sign*((-(d / 2)*cos(s) + d / 2)*cos(alpha)
					+ walk_sign*beginPos[i + walk_axis] * cos((1 - cos(s)) / 2 * beta)
					- left_sign*beginPos[i + side_axis] * sin((1 - cos(s)) / 2 * beta));
				pEE[i + up_axis] = up_sign*h*sin(s)
					+ beginPos[i + up_axis];
				pEE[i + side_axis] = left_sign*((-(d / 2)*cos(s) + d / 2)*sin(alpha)
					+ walk_sign*beginPos[i + walk_axis] * sin((1 - cos(s)) / 2 * beta)
					+ left_sign*beginPos[i + side_axis] * cos((1 - cos(s)) / 2 * beta));
			}
		}
		
		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		double pr[6] = { 0 }, pe[6];
		double pm[4][4];
		pr[up_axis+3] = up_sign*beta*(count+1)/totalCount/2;
		s_pr2pm(pr, *pm);
		s_pm2pe(*pm, pe);

		memcpy(pBodyPE, pe, sizeof(double) * 6);

		pBodyPE[walk_axis] = walk_sign*(d*(t / T)*cos(alpha) + tan(beta / 2)*d*t / T*(1 - t / T)*sin(alpha));
		pBodyPE[up_axis] = 0;
		pBodyPE[side_axis] = left_sign*(d*(t / T)*sin(alpha) - tan(beta / 2)*d*t / T*(1 - t / T)*cos(alpha));

		






		pRobot->SetPee(pEE, pBodyPE);
		pRobot->GetPee(pEE, "B");
		pRobot->GetPin(pIn);

		return 2 * totalCount - count - 1;
	}

	
	
	int move(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned totalCount,
		const double *moveDirection,
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
		double *pBodyEp)
	{


		return totalCount - count + 1;
	}

	int moveLeg(
		ROBOT_BASE *pRobot,
		unsigned count,
		unsigned totalCount,
		const double *beginPee,
		const double *beginBodyEp,
		const double *endPee,
		const unsigned *legID,
		int upAxis,
		double *pIn,
		double *pEE,
		double *pBodyEp)
	{
		return totalCount - count + 1;
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
