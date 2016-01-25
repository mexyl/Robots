

#ifdef UNIX
#include "rtdk.h"
#endif
#ifdef WIN32
#define rt_printf printf
#endif

#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

#include <aris.h>

#include "Robot_Gait.h"
#include "Robot_Base.h"


using namespace Aris::Dynamic;

namespace Robots
{
	int walkAcc(RobotBase * pRobot, const GaitParamBase * pParam)
	{
		/*初始化参数*/
		const WalkParam *pRealParam = static_cast<const WalkParam *>(pParam);

		int wAxis = std::abs(pRealParam->walkDirection) - 1;
		int uAxis = std::abs(pRealParam->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginPeb, *pm, "313");
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
		const double *beginPeb = pRealParam->beginPeb;

		double pEE[18];
		double pBodyPE[6];

		/*初始化完毕，开始计算*/
		int count = pRealParam->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
		/*设置移动腿*/
		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a + b * 3 / 4)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * cos((1 - cos(s)) / 4 * b)
				- lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * sin((1 - cos(s)) / 4 * b))
				+ beginPeb[wAxis];
			pEE[i + uAxis] = uSign*h*sin(s)
				+ beginPee[i + uAxis];
			pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a + b * 3 / 4)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * sin((1 - cos(s)) / 4 * b)
				+ lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * cos((1 - cos(s)) / 4 * b))
				+ beginPeb[lAxis];
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
		pRobot->SetPeb(pBodyPE);
		pRobot->SetPee(pEE);
		return totalCount - count - 1;
	}
	int walkConst(RobotBase * pRobot, const GaitParamBase * pParam)
	{
		/*初始化参数*/
		const WalkParam *pRealParam = static_cast<const WalkParam *>(pParam);

		int wAxis = std::abs(pRealParam->walkDirection) - 1;
		int uAxis = std::abs(pRealParam->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginPeb, *pm, "313");
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
		const double *beginPeb = pRealParam->beginPeb;

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
					+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * cos((1 - cos(s)) / 2 * b)
					- lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * sin((1 - cos(s)) / 2 * b))
					+ beginPeb[wAxis];
				pEE[i + uAxis] = uSign*h*sin(s)
					+ beginPee[i + uAxis];
				pEE[i + lAxis] = lSign*(d *sin(a + b)*(1 - cos(s)) / 2
					+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * sin((1 - cos(s)) / 2 * b)
					+ lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * cos((1 - cos(s)) / 2 * b))
					+ beginPeb[lAxis];
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
					+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * cos((1 - cos(s)) / 2 * b)
					- lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * sin((1 - cos(s)) / 2 * b))
					+ beginPeb[wAxis];
				pEE[i + uAxis] = uSign*h*sin(s)
					+ beginPee[i + uAxis];
				pEE[i + lAxis] = lSign*(d *sin(a + b)*(1 - cos(s)) / 2
					+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * sin((1 - cos(s)) / 2 * b)
					+ lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * cos((1 - cos(s)) / 2 * b))
					+ beginPeb[lAxis];
			}
			/*设置支撑腿*/
			for (int i = 3; i < 18; i += 6)
			{
				pEE[i + wAxis] = wSign*(d *cos(a + b)
					+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * cos(b)
					- lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * sin(b))
					+ beginPeb[wAxis];
				pEE[i + uAxis] = 0
					+ beginPee[i + uAxis];
				pEE[i + lAxis] = lSign*(d *sin(a + b)
					+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis]) * sin(b)
					+ lSign*(beginPee[i + lAxis] - beginPeb[lAxis]) * cos(b))
					+ beginPeb[lAxis];
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
		pRobot->SetPeb(pBodyPE);
		pRobot->SetPee(pEE);
		return 2 * totalCount - pRealParam->count - 1;
	}
	int walkDec(RobotBase * pRobot, const GaitParamBase * pParam)
	{
		/*初始化参数*/
		const WalkParam *pRealParam = static_cast<const WalkParam *>(pParam);

		int wAxis = std::abs(pRealParam->walkDirection) - 1;
		int uAxis = std::abs(pRealParam->upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
		int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

		double pm[4][4], pe[6];
		s_pe2pm(pParam->beginPeb, *pm, "313");
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
		const double *beginPeb = pRealParam->beginPeb;

		double pEE[18];
		double pBodyPE[6];

		/*初始化完毕，开始计算*/
		int count = pRealParam->count;
		double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
		/*设置移动腿*/
		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a + b / 2)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a + b / 2)) * cos((1 - cos(s)) / 4 * b)
				- lSign*(beginPee[i + lAxis] - beginPeb[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a + b / 2)) * sin((1 - cos(s)) / 4 * b))
				+ beginPeb[wAxis] - wSign*0.25*d / cos(b / 2)*cos(a+b/2);
			pEE[i + uAxis] = uSign*h*sin(s)
				+ beginPee[i + uAxis];
			pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a + b / 2)*(1 - cos(s)) / 2
				+ wSign*(beginPee[i + wAxis] - beginPeb[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a + b / 2)) * sin((1 - cos(s)) / 4 * b)
				+ lSign*(beginPee[i + lAxis] - beginPeb[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a + b / 2)) * cos((1 - cos(s)) / 4 * b))
				+ beginPeb[lAxis] - lSign*0.25*d / cos(b / 2)*sin(a + b / 2);
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
		pRobot->SetPeb(pBodyPE);
		pRobot->SetPee(pEE);
		return totalCount - count - 1;
	}
	int walk(Aris::Dynamic::ModelBase &model, const Aris::Dynamic::PlanParamBase &param_in)
	{
		auto &robot = static_cast<Robots::RobotBase &>(model);
		auto &param = static_cast<const Robots::WalkParam &>(param_in);

		/*初始化*/
		static Aris::Dynamic::FloatMarker beginMak{ robot.Ground() };
		static double beginPee[18];
		if (param.count%param.totalCount == 0) 
		{ 
			beginMak.setPrtPm(*robot.Body().pm());
			beginMak.update();
			robot.GetPee(beginPee, beginMak); 
		}

		int wAxis = std::abs(param.walkDirection) - 1;
		int uAxis = std::abs(param.upDirection) - 1;
		int lAxis = 3 - wAxis - uAxis;
		int wSign = param.walkDirection / std::abs(param.walkDirection);
		int uSign = param.upDirection / std::abs(param.upDirection);
		int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;


		/*以下设置各个阶段的身体的真实初始位置*/
		const double a = param.alpha;
		const double b = param.beta;

		char order[4];
		order[0] = '1' + uAxis;
		order[1] = '1' + (1 + uAxis) % 3;
		order[2] = '1' + (2 + uAxis) % 3;

		

		return 2 * param.n * param.totalCount - param.count - 1;
	}
	Aris::Core::Msg parseWalk(const std::string &cmd, const std::map<std::string, std::string> &params)
	{
		Robots::WalkParam  param;

		for (auto &i : params)
		{
			if (i.first == "totalCount")
			{
				param.totalCount = std::stoi(i.second);
			}
			else if (i.first == "n")
			{
				param.n = stoi(i.second);
			}
			else if (i.first == "walkDirection")
			{
				param.walkDirection = stoi(i.second);
			}
			else if (i.first == "upDirection")
			{
				param.upDirection = stoi(i.second);
			}
			else if (i.first == "distance")
			{
				param.d = stod(i.second);
			}
			else if (i.first == "height")
			{
				param.h = stod(i.second);
			}
			else if (i.first == "alpha")
			{
				param.alpha = stod(i.second);
			}
			else if (i.first == "beta")
			{
				param.beta = stod(i.second);
			}
		}

		Aris::Core::Msg msg;

		msg.copyStruct(param);

		return msg;
	}

	int fastWalk(RobotBase * pRobot, const GaitParamBase * pParam)
	{
		auto pFP = static_cast<const FastWalkParam*>(pParam);
		
		if (pFP->count < pFP->accCount)
		{
			pRobot->SetPin(&pFP->pInAcc[pFP->count * 18]);
		}
		else if ((pFP->count- pFP->accCount) < ((pFP->n - 1) * pFP->constCount))
		{
			int count = (pFP->count - pFP->accCount) % pFP->constCount;
			pRobot->SetPin(&pFP->pInConst[count * 18]);
		}
		else
		{
			int count = pFP->count - pFP->accCount - ((pFP->n - 1) * pFP->constCount);
			pRobot->SetPin(&pFP->pInDec[count * 18]);
		}

		return pFP->accCount + ((pFP->n - 1) * pFP->constCount) + pFP->decCount - pFP->count-1;
	}
	Aris::Core::Msg parseFastWalk(const std::string &cmd, const std::map<std::string, std::string> &params)
	{
		FastWalkParam param;
		for (auto &i : params)
		{
			if (i.first == "file")
			{
				static std::map<std::string, std::tuple<int, int, int, std::unique_ptr<double> > > walkFileMap;

				const auto found = walkFileMap.find(i.second);
				if (found != walkFileMap.end())
				{
					/*步态已经存在*/
					std::tie(param.accCount, param.constCount, param.decCount, std::ignore) = found->second;
					param.pInAcc = std::get<3>(found->second).get();
					param.pInConst = std::get<3>(found->second).get() + 18* param.accCount;
					param.pInDec = std::get<3>(found->second).get() + 18 * (param.accCount + param.constCount);

					//std::cout << "acc count:" << param.accCount << std::endl;
					//std::cout << "const count:" << param.constCount << std::endl;
					//std::cout << "dec count:" << param.decCount << std::endl;
				}
				else
				{
					if (walkFileMap.size() > 1)
					{
						throw std::runtime_error("only one path is allowed");
					}
					
					/*插入步态*/
					std::ifstream file;
					std::string accFile = i.second + "_acc.txt";
					std::string constFile = i.second + "_const.txt";
					std::string decFile = i.second + "_dec.txt";

					int accNum{ -1 }, decNum{ -1 }, constNum{ -1 };

					file.open(accFile);
					if (!file) throw std::logic_error("acc file not exist");
					for (double tem; !file.eof(); file >> tem) ++accNum;
					if (accNum % 18 != 0) throw std::logic_error("acc file invalid, because the num of numbers is not valid");
					accNum /= 18;
					file.close();

					file.open(constFile);
					if (!file) throw std::logic_error("const file not exist");
					for (double tem; !file.eof(); file >> tem) ++constNum;
					if (constNum % 18 != 0) throw std::logic_error("const file invalid, because the num of numbers is not valid");
					constNum = constNum / 18;
					file.close();

					file.open(decFile);
					if (!file) throw std::logic_error("dec file not exist");
					for (double tem; !file.eof(); file >> tem) ++decNum;
					if (decNum % 18 != 0) throw std::logic_error("dec file invalid, because the num of numbers is not valid");
					decNum = decNum / 18;
					file.close();

					param.accCount = accNum;
					param.constCount = constNum;
					param.decCount = decNum;

					std::unique_ptr<double> p(new double[(accNum + constNum + decNum) * 18]);

					param.pInAcc = p.get();
					param.pInConst = p.get() + accNum * 18;
					param.pInDec = p.get() + (accNum + constNum) * 18;

					file.open(accFile);
					for (int i = 0; !file.eof(); file >> param.pInAcc[i++]);
					file.close();

					file.open(constFile);
					for (int i = 0; !file.eof(); file >> param.pInConst[i++]);
					file.close();

					file.open(decFile);
					for (int i = 0; !file.eof(); file >> param.pInDec[i++]);
					file.close();

					walkFileMap.insert(std::make_pair(i.second, std::make_tuple(accNum, constNum, decNum, std::move(p))));
					
				}
			}
			else if (i.first == "n")
			{
				param.n = std::stoi(i.second);
			}
			else
			{
				throw std::logic_error("internal error happened, because invalid params in parseFastWalk");
			}
		}
		
		Aris::Core::Msg msg;

		msg.copyStruct(param);

		return msg;
		
	}

	int resetOrigin(RobotBase * pRobot, const Robots::GaitParamBase *pParam)
	{
		if (pParam->imuData)
		{
			double eul[3];
			pParam->imuData->toEulBody2Ground(eul,PI,"321");			
			rt_printf("imu eul321: %f,%f,%f\n", eul[0], eul[1], eul[2]);
		}
		else
		{
			rt_printf("cannot find imu\n");
		}

		if (pParam->pForceData->empty())
		{
			rt_printf("cannot find force sensor\n");
		}
		else
		{
			rt_printf("force are:\n%f %f %f %f %f %f\n"
				, pParam->pForceData->at(0).Fx
				, pParam->pForceData->at(0).Fy
				, pParam->pForceData->at(0).Fz
				, pParam->pForceData->at(0).Mx
				, pParam->pForceData->at(0).My
				, pParam->pForceData->at(0).Mz);
		}

		
		
		double pEE[18], pBody[6]{ 0 }, vEE[18], vBody[6]{ 0 };
		pRobot->GetPee(pEE, pRobot->Body());
		pRobot->GetVee(vEE, pRobot->Body());

		pRobot->SetPeb(pBody);
		pRobot->SetPee(pEE);

		pRobot->SetVb(vBody);
		pRobot->SetVee(vEE);

		return 0;
	}
	Aris::Core::Msg parseResetOrigin(const std::string &cmd, const std::map<std::string, std::string> &params)
	{
		Robots::GaitParamBase param;
		Aris::Core::Msg msg;
		msg.copyStruct(param);
		return msg;
	}
}
