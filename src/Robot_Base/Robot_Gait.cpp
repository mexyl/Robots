

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
	int walk(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param_in)
	{
		auto &robot = static_cast<Robots::RobotBase &>(model);
		auto &param = static_cast<const Robots::WalkParam &>(param_in);

		/*初始化*/
		static Aris::Dynamic::FloatMarker beginMak{ robot.ground() };
		static double beginPee[18];

		if (param.count%param.totalCount == 0)
		{
			beginMak.setPrtPm(*robot.Body().pm());
			beginMak.update();
			robot.GetPee(beginPee, beginMak);
		}

		double front[3]{ 0,0,-1 };
		double left[3]{ -1,0,0 };
		double up[3]{ 0,1,0 };

		/*以下设置各个阶段的身体的真实初始位置*/
		const double a = param.alpha;
		const double b = param.beta;
		const double d = param.d;
		const double h = param.h;

		const double r = d / 2 / std::sin(b / 2);

		int period_count = param.count%param.totalCount;
		const double s = -(PI / 2)*cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI. 

		double Peb[6], Pee[18];
		std::fill(Peb, Peb + 6, 0);
		std::copy(beginPee, beginPee + 18, Pee);


		double pq_b[7]{ 0,0,0,std::sin(b / 2)*up[0],std::sin(b / 2)*up[1],std::sin(b / 2)*up[2],std::cos(b / 2) };
		double pq_b_half[7]{ 0,0,0,std::sin(b / 4)*up[0],std::sin(b / 4)*up[1],std::sin(b / 4)*up[2],std::cos(b / 4) };
		double pq_b_quad[7]{ 0,0,0,std::sin(b / 8)*up[0],std::sin(b / 8)*up[1],std::sin(b / 8)*up[2],std::cos(b / 8) };
		double pq_b_eighth[7]{ 0,0,0,std::sin(b / 16)*up[0],std::sin(b / 16)*up[1],std::sin(b / 16)*up[2],std::cos(b / 16) };
		double pm_b[16], pm_b_half[16], pm_b_quad[16], pm_b_eighth[16];
		
		s_pq2pm(pq_b, pm_b);
		s_pq2pm(pq_b_half, pm_b_half);
		s_pq2pm(pq_b_quad, pm_b_quad);
		s_pq2pm(pq_b_eighth, pm_b_eighth);

		const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;

		if ((param.count / param.totalCount) == 0)//加速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

				s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
				s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
				s_daxpy(3, d/2, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double body_forward_dir[3], body_left_dir[3];
			s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
			s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * s_interp(param.totalCount, period_count+1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 4))
					+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 4));
			}

			//规划身体姿态
			double s_acc = Aris::Dynamic::acc_even(param.totalCount, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_acc*b / 8)*up[0],std::sin(s_acc*b / 8)*up[1] ,std::sin(s_acc*b / 8)*up[2],std::cos(s_acc*b / 8) };
			double pe[6];
			s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}
		else if ((param.count / param.totalCount) == (param.n * 2 - 1))//减速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

				s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
				s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
				s_daxpy(3, d / 2, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double body_forward_dir[3], body_left_dir[3];
			s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
			s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * s_interp(param.totalCount, period_count+1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, 0)
					+ front[i] * s_interp(param.totalCount, period_count+1, 0, d / 4 / std::cos(b / 4), d/2 / param.totalCount / std::cos(b / 2),0);
			}

			//规划身体姿态
			double s_dec = Aris::Dynamic::dec_even(param.totalCount, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_dec*b / 8)*up[0],std::sin(s_dec*b / 8)*up[1] ,std::sin(s_dec*b / 8)*up[2],std::cos(s_dec*b / 8) };
			double pe[6];
			s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}
		else//匀速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				s_pm_dot_v3(pm_b_half, front, leg_forward_dir);
				
				s_pm_dot_v3(pm_b, beginPee + i, forward_d);
				s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
				s_daxpy(3, d, leg_forward_dir, 1, forward_d, 1);
				
				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double d2 = d / 2 / std::cos(b / 4);
			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * s_interp(param.totalCount, period_count + 1, 0, d2*std::sin(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 2))
					+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d/2, d / 2 / param.totalCount / std::cos(b / 2), d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b/2));
			}

			//规划身体姿态
			double s_even = even(param.totalCount, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_even*b / 4)*up[0],std::sin(s_even*b / 4)*up[1] ,std::sin(s_even*b / 4)*up[2],std::cos(s_even*b / 4) };
			double pe[6];
			s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}

		robot.SetPeb(Peb, beginMak);
		robot.SetPee(Pee, beginMak);

		//static std::list<std::array<double, 6> > PebList;
		//static std::list<std::array<double, 18> > PeeList, PinList;

		//std::array<double, 6> Peb_data;
		//std::array<double, 18> Pee_data, Pin_data;
		//std::copy(Peb, Peb + 6, Peb_data.data());

		//robot.GetPeb(Peb_data.data());
		//robot.GetPin(Pin_data.data());
		//robot.GetPee(Pee_data.data());

		//PebList.push_back(Peb_data);
		//PeeList.push_back(Pee_data);
		//PinList.push_back(Pin_data);

		
		//if (2 * param.n * param.totalCount - param.count - 1 == 0)
		//{
		//	Aris::Dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\Peb.txt", PebList);
		//	Aris::Dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\Pee.txt", PeeList);
		//	Aris::Dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\Pin.txt", PinList);
		//}
			

		return 2 * param.n * param.totalCount - param.count - 1;
	}
	void parseWalk(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)
	{
		Robots::WalkParam param;

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
		msg.copyStruct(param);
	}


}
