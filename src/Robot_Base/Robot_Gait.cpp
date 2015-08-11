#include <Platform.h>

#include <cstring>
#include <cmath>
#include <algorithm>
#include <Aris_DynKer.h>
#include <Aris_Plan.h>

#include "Robot_Gait.h"
#include "Robot_Base.h"


using namespace Aris::Plan;
using namespace Aris::DynKer;

namespace Robots
{
	int walkAcc(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		/*初始化参数*/
		const WALK_PARAM *pRealParam = static_cast<const WALK_PARAM *>(pParam);

		int wAxis = std::abs(pRealParam->walkDirection) - 1;
		int uAxis = std::abs(pRealParam->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginBodyPE, *pm, "313");
		char order[4]{0};
		order[0] = '1' + uAxis;
		order[1] = '1' + (1 + uAxis) % 3;
		order[2] = '1' + (2 + uAxis) % 3;
		s_pm2pe(*pm, pe, order);

		int totalCount = pRealParam->totalCount;
		double h = pRealParam->h;
		double d = pRealParam->d;
		double a = pRealParam->alpha + uSign*pe[3];
		double b = pRealParam->beta;

		const double *beginPee = pRealParam->beginPee;
		const double *beginBodyPE = pRealParam->beginBodyPE;

		double pEE[18];
		double pBodyPE[6];

		/*初始化完毕，开始计算*/
		int count = pRealParam->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
		/*设置移动腿*/
		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a + b * 3 / 4)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos((1 - cos(s)) / 4 * b)
				- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin((1 - cos(s)) / 4 * b))
				+ beginBodyPE[wAxis];
			pEE[i + uAxis] = uSign*h*sin(s)
				+ beginPee[i + uAxis];
			pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a + b * 3 / 4)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin((1 - cos(s)) / 4 * b)
				+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos((1 - cos(s)) / 4 * b))
				+ beginBodyPE[lAxis];
		}
		/*设置支撑腿*/
		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + wAxis] = beginPee[i + wAxis];
			pEE[i + uAxis] = beginPee[i + uAxis];
			pEE[i + lAxis] = beginPee[i + lAxis];
		}

		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		/*以下计算角度，需要在313和不同的欧拉角中间转来转去*/
		pe[3]+= uSign*b / 4 * acc_even(totalCount, count + 1);
		s_pe2pm(pe, *pm, order);
		s_pm2pe(*pm, pBodyPE);

		/*以下计算位置*/
		pBodyPE[wAxis] += wSign*0.25*d / cos(b / 2)*cos(a + b * 3 / 4)*(acc_even(totalCount, count + 1));
		pBodyPE[uAxis] += 0;
		pBodyPE[lAxis] += lSign*0.25*d / cos(b / 2)*sin(a + b * 3 / 4)*(acc_even(totalCount, count + 1));

		/*计算完毕，更新pRobot*/
		pRobot->SetPee(pEE, pBodyPE, "G");
		return totalCount - count - 1;
	}
	int walkConst(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		/*初始化参数*/
		const WALK_PARAM *pRealParam = static_cast<const WALK_PARAM *>(pParam);

		int wAxis = std::abs(pRealParam->walkDirection) - 1;
		int uAxis = std::abs(pRealParam->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginBodyPE, *pm, "313");
		char order[4]{ 0 };
		order[0] = '1' + uAxis;
		order[1] = '1' + (1 + uAxis) % 3;
		order[2] = '1' + (2 + uAxis) % 3;
		s_pm2pe(*pm, pe, order);

		int totalCount = pRealParam->totalCount;
		double h = pRealParam->h;
		double d = pRealParam->d;
		double a = pRealParam->alpha + uSign*pe[3];
		double b = pRealParam->beta;

		const double *beginPee = pRealParam->beginPee;
		const double *beginBodyPE = pRealParam->beginBodyPE;

		double pEE[18];
		

		/*初始化完毕，开始计算*/
		if (pRealParam->count < pRealParam->totalCount)
		{
			int count = pRealParam->count;
			double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
			/*设置移动腿*/
			for (int i = 3; i < 18; i += 6)
			{
				pEE[i + wAxis] = wSign*(d *cos(a + b)*(1 - cos(s)) / 2
					+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos((1 - cos(s)) / 2 * b)
					- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin((1 - cos(s)) / 2 * b))
					+ beginBodyPE[wAxis];
				pEE[i + uAxis] = uSign*h*sin(s)
					+ beginPee[i + uAxis];
				pEE[i + lAxis] = lSign*(d *sin(a + b)*(1 - cos(s)) / 2
					+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin((1 - cos(s)) / 2 * b)
					+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos((1 - cos(s)) / 2 * b))
					+ beginBodyPE[lAxis];
			}
			/*设置支撑腿*/
			for (int i = 0; i < 18; i += 6)
			{
				pEE[i + wAxis] = beginPee[i + wAxis];
				pEE[i + uAxis] = beginPee[i + uAxis];
				pEE[i + lAxis] = beginPee[i + lAxis];
			}
		}
		else
		{
			int count = pRealParam->count - pRealParam->totalCount;
			double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
			/*设置移动腿*/
			for (int i = 0; i < 18; i += 6)
			{
				pEE[i + wAxis] = wSign*(d *cos(a + b)*(1 - cos(s)) / 2
					+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos((1 - cos(s)) / 2 * b)
					- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin((1 - cos(s)) / 2 * b))
					+ beginBodyPE[wAxis];
				pEE[i + uAxis] = uSign*h*sin(s)
					+ beginPee[i + uAxis];
				pEE[i + lAxis] = lSign*(d *sin(a + b)*(1 - cos(s)) / 2
					+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin((1 - cos(s)) / 2 * b)
					+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos((1 - cos(s)) / 2 * b))
					+ beginBodyPE[lAxis];
			}
			/*设置支撑腿*/
			for (int i = 3; i < 18; i += 6)
			{
				pEE[i + wAxis] = wSign*(d *cos(a + b)
					+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos(b)
					- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin(b))
					+ beginBodyPE[wAxis];
				pEE[i + uAxis] = uSign*h*sin(s)
					+ beginPee[i + uAxis];
				pEE[i + lAxis] = lSign*(d *sin(a + b)
					+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin(b)
					+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos(b))
					+ beginBodyPE[lAxis];
			}
		}

		


		/*设置身体*/
		double t = pRealParam->count + 1;
		double T = totalCount * 2;

		/*以下计算角度，需要在313和不同的欧拉角中间转来转去*/
		double pBodyPE[6];
		pe[3] += uSign*b * even(totalCount * 2, pRealParam->count + 1);
		s_pe2pm(pe, *pm, order);
		s_pm2pe(*pm, pBodyPE);

		/*以下计算位置*/
		
		double s = even(totalCount * 2, pRealParam->count + 1);

		pBodyPE[wAxis] += wSign*(d *s*cos(a + b) + tan(b / 2) * d  * sin(a + b) * (s - s*s));
		pBodyPE[uAxis] += 0;
		pBodyPE[lAxis] += lSign*(d *s*sin(a + b) - tan(b / 2) * d  * cos(a + b) * (s - s*s));


		/*计算完毕，更新pRobot*/
		pRobot->SetPee(pEE, pBodyPE, "G");
		return 2 * totalCount - pRealParam->count - 1;
	}
	int walkDec(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		/*初始化参数*/
		const WALK_PARAM *pRealParam = static_cast<const WALK_PARAM *>(pParam);

		int wAxis = std::abs(pRealParam->walkDirection) - 1;
		int uAxis = std::abs(pRealParam->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginBodyPE, *pm, "313");
		char order[4]{0};
		order[0] = '1' + uAxis;
		order[1] = '1' + (1 + uAxis) % 3;
		order[2] = '1' + (2 + uAxis) % 3;
		s_pm2pe(*pm, pe, order);

		int totalCount = pRealParam->totalCount;
		double h = pRealParam->h;
		double d = pRealParam->d;
		double a = pRealParam->alpha + uSign*pe[3];
		double b = pRealParam->beta;

		const double *beginPee = pRealParam->beginPee;
		const double *beginBodyPE = pRealParam->beginBodyPE;

		double pEE[18];
		double pBodyPE[6];

		/*初始化完毕，开始计算*/
		int count = pRealParam->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
		/*设置移动腿*/
		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a + b / 2)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a + b / 2)) * cos((1 - cos(s)) / 4 * b)
				- lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a + b / 2)) * sin((1 - cos(s)) / 4 * b))
				+ beginBodyPE[wAxis] - wSign*0.25*d / cos(b / 2)*cos(a+b/2);
			pEE[i + uAxis] = uSign*h*sin(s)
				+ beginPee[i + uAxis];
			pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a + b / 2)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a + b / 2)) * sin((1 - cos(s)) / 4 * b)
				+ lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a + b / 2)) * cos((1 - cos(s)) / 4 * b))
				+ beginBodyPE[lAxis] - lSign*0.25*d / cos(b / 2)*sin(a + b / 2);
		}



		/*设置支撑腿*/
		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + wAxis] = beginPee[i + wAxis];
			pEE[i + uAxis] = beginPee[i + uAxis];
			pEE[i + lAxis] = beginPee[i + lAxis];
		}

		/*设置身体*/
		double t = count + 1;
		double T = totalCount * 2;

		/*以下计算角度，需要在313和321的欧拉角中间转来转去*/
		pe[3] += uSign*b / 4 * dec_even(totalCount, count + 1);
		s_pe2pm(pe, *pm, order);
		s_pm2pe(*pm, pBodyPE);

		/*以下计算位置*/
		pBodyPE[wAxis] += wSign*0.25*d / cos(b / 2)*cos(a + b / 2)*(dec_even(totalCount, count + 1));
		pBodyPE[uAxis] += 0;
		pBodyPE[lAxis] += lSign*0.25*d / cos(b / 2)*sin(a + b / 2)*(dec_even(totalCount, count + 1));

		/*计算完毕，更新pRobot*/
		pRobot->SetPee(pEE, pBodyPE, "G");
		return totalCount - count - 1;
	}

	


	int walk(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		const Robots::WALK_PARAM *pWP = static_cast<const Robots::WALK_PARAM *>(pParam);

		/*以下设置各个阶段的身体的真实初始位置*/
		double beginPm[16];
		s_pe2pm(pWP->beginBodyPE, beginPm);
	
		int wAxis = std::abs(pWP->walkDirection) - 1;
		int uAxis = std::abs(pWP->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pWP->walkDirection / std::abs(pWP->walkDirection);
		int uSign = pWP->upDirection / std::abs(pWP->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		const double a = pWP->alpha;
		const double b = pWP->beta;

		char order[4];
		order[0] = '1' + uAxis;
		order[1] = '1' + (1 + uAxis) % 3;
		order[2] = '1' + (2 + uAxis) % 3;

		double peFirstStep[6]{ 0,0,0,uSign*b / 4,0,0 };
		peFirstStep[wAxis] =
			wSign *pWP->d / cos(b / 2) / 4 * cos(b * 3 / 4 + a);
		peFirstStep[lAxis] =
			lSign *pWP->d / cos(b / 2) / 4 * sin(b * 3 / 4 + a);
		double pmFirst[16];
		s_pe2pm(peFirstStep, pmFirst, order);


		int stepCount = (pParam->count - pWP->totalCount) / (2 * pWP->totalCount);
		double theta = stepCount*b;
		double peConstStep[6]{ 0,0,0,uSign*theta,0,0 };
		if (std::abs(b) > 1e-10)
		{
			double r = pWP->d / sin(b / 2) / 2;
			peConstStep[wAxis] =
				wSign *(-r*sin(b / 2 + a) + r * sin(theta + b / 2 + a));
			peConstStep[lAxis] =
				+lSign *(r*cos(b / 2 + a) - r * cos(theta + b / 2 + a));
		}
		else
		{
			peConstStep[wAxis] = wSign * pWP->d * stepCount * cos(a);
			peConstStep[lAxis] = lSign * pWP->d * stepCount * sin(a);
		}
		double pmConst[16];
		s_pe2pm(peConstStep, pmConst, order);

		double pm1[16], bodyRealBeginPm[16];
		s_pm_dot_pm(beginPm, pmFirst, pm1);
		s_pm_dot_pm(pm1, pmConst, bodyRealBeginPm);

		/*将身体初始位置写入参数*/
		Robots::WALK_PARAM realParam = *pWP;

		realParam.count = (pWP->count - pWP->totalCount) % (2 * pWP->totalCount);
		s_pm2pe(bodyRealBeginPm, realParam.beginBodyPE);
		
		/*以下计算每个腿的初始位置*/
		double pee_G[18]{ 0 };
		double pee_B[18]{ 0 };

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginBodyPE, *pm, "313");
		s_pm2pe(*pm, pe, order);

		for (int i = 0; i < 18; i += 3)
		{
			if ((i/3) % 2 == 0)
			{
				pee_G[i + wAxis] = wSign*(0.5* pWP->d / cos(b / 2)*cos(a+ uSign*pe[3] + b * 3 / 4)
					+ wSign*(pWP->beginPee[i + wAxis] - pWP->beginBodyPE[wAxis]) * cos(b / 2)
					- lSign*(pWP->beginPee[i + lAxis] - pWP->beginBodyPE[lAxis]) * sin(b / 2))
					+ pWP->beginBodyPE[wAxis];
				pee_G[i + uAxis] = pWP->beginPee[i + uAxis];
				pee_G[i + lAxis] = lSign*(0.5*pWP->d / cos(b / 2)*sin(a + uSign*pe[3] + b * 3 / 4)
					+ wSign*(pWP->beginPee[i + wAxis] - pWP->beginBodyPE[wAxis]) * sin(b / 2)
					+ lSign*(pWP->beginPee[i + lAxis] - pWP->beginBodyPE[lAxis]) * cos(b / 2))
					+ pWP->beginBodyPE[lAxis];
			}
			else
			{
				std::copy_n(pWP->beginPee + i, 3, pee_G + i);
			}

			s_inv_pm_dot_pnt(pm1, pee_G + i, pee_B + i);
			s_pm_dot_pnt(bodyRealBeginPm, pee_B + i, realParam.beginPee + i);
		}

		//static double lastPee[18], lastPbody[6];
		//std::copy_n(lastPee, 18, realParam.beginPee);
		//std::copy_n(lastPbody, 6, realParam.beginBodyPE);


		if (pParam->count < pWP->totalCount)
		{
			walkAcc(pRobot, pParam);
		}
		else if (pParam->count < (pWP->n*2-1) * pWP->totalCount)
		{
			walkConst(pRobot, &realParam);
		}
		else
		{
			walkDec(pRobot, &realParam);
		}

		/*if ((pParam->count + pWP->totalCount + 1) %(2*pWP->totalCount)==0 )
		{
			pRobot->GetPee(lastPee);
			pRobot->GetBodyPe(lastPbody);
		}*/

		return 2 * pWP->n * pWP->totalCount - pWP->count - 1;
	}
	int adjust(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		auto pAP = static_cast<const ADJUST_PARAM*>(pParam);

		int pos, periodBeginCount{ 0 }, periodEndCount{ 0 };

		for (int i = 0; i < pAP->periodNum; ++i)
		{
			periodEndCount += pAP->periodCount[i];
			
			if ((pAP->count < periodEndCount) && (pAP->count >= periodBeginCount))
			{
				pos = i;
				break;
			}
			
			periodBeginCount = periodEndCount;
		}

		double s = -(PI / 2)*cos(PI * (pAP->count - periodBeginCount + 1) / (periodEndCount- periodBeginCount)) + PI / 2;
		
		double pEE[18], pBody[6];

		if (pos == 0)
		{
			for (int i = 0; i < 18; ++i)
			{
				pEE[i] = pAP->beginPee[i] * (cos(s) + 1) / 2 + pAP->targetPee[pos][i] * (1 - cos(s)) / 2;
				
			}
			for (int i = 0; i < 6; ++i)
			{
				pBody[i] = pAP->beginBodyPE[i] * (cos(s) + 1) / 2 + pAP->targetBodyPE[pos][i] * (1 - cos(s)) / 2;
			}
		}
		else
		{
			for (int i = 0; i < 18; ++i)
			{
				pEE[i] = pAP->targetPee[pos-1][i] * (cos(s) + 1) / 2 + pAP->targetPee[pos][i] * (1 - cos(s)) / 2;
				
			}
			for (int i = 0; i < 6; ++i)
			{
				pBody[i] = pAP->targetBodyPE[pos - 1][i] * (cos(s) + 1) / 2 + pAP->targetBodyPE[pos][i] * (1 - cos(s)) / 2;
			}
		}

		pRobot->SetPee(pEE, pBody);

		int totalCount{ 0 };
		for (int i = 0; i < pAP->periodNum; ++i)
		{
			totalCount += pAP->periodCount[i];
		}

		return totalCount - pAP->count - 1;
	}
	
	
	
	int move(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
	{
		const Robots::MOVE_PARAM *param = static_cast<const Robots::MOVE_PARAM *>(pParam);

		double beginPq[7], beginVq[7], endPq[7], endVq[7];
		
		/*计算末端位置和速度在初始位置下的相对值*/
		double relativeBeginPm[16], relativeBeginPq[7]{0,0,0,0,0,0,1};
		double relativeBeginVel[6], relativeBeginVq[6];
		double relativeEndPm[16], relativeEndPq[7];
		double relativeEndVel[6], relativeEndVq[6];

		/*begin pm and pq*/
		s_pq2pm(relativeBeginPq, relativeBeginPm);

		/*begin vq*/
		double pm1[16], pm2[16];
		s_pe2pm(param->beginBodyPE, pm1);
		s_inv_v2v(pm1, nullptr, param->beginBodyVel, relativeBeginVel);
		s_v2vq(relativeBeginPm, relativeBeginVel, relativeBeginVq);

		/*end pq*/
		s_pe2pm(param->targetBodyPE, pm2);
		s_inv_pm_dot_pm(pm1, pm2, relativeEndPm);

		/*end vq*/
		s_inv_v2v(pm1, nullptr, param->beginBodyVel, relativeEndVel);
		




		//s_v2v(nullptr,nullptr);
		//Aris::DynKer::s_pm2pq();



		



		param->beginBodyVel;






		return 0;
	}
}
