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

	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		auto pAP = static_cast<const ADJUST_PARAM*>(pParam);

		unsigned count = pAP->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / pAP->totalCount) + PI / 2;

		double pEE[18], pBody[6];

		for (int i = 0; i < 18; ++i)
		{
			pEE[i] = pAP->beginPee[i] * (cos(s) + 1) / 2 + pAP->targetPee[i] * (1 - cos(s)) / 2;
		}

		for (int i = 0; i < 6; ++i)
		{
			pBody[i] = pAP->beginBodyPE[i] * (cos(s) + 1) / 2 + pAP->targetBodyPE[i] * (1 - cos(s)) / 2;
		}


		pRobot->SetPee(pEE, pBody);

		return pAP->totalCount - pAP->count - 1;
	}
}
