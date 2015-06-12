#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <Aris_DynKer.h>
#include <fstream>



//需要修改默认的堆栈大小
//在windows里，位于程序属性->链接器->系统->堆栈保留大小

/** \brief 命名空间：六足机器人
*
* 用来计算机器人的运动学和动力学等。
*/
namespace Robots
{
	class ROBOT_BASE;
	
	class LEG_BASE
	{
		friend class ROBOT_BASE;

	private:
		

		double _BasePrtPm[4][4];
		double _BasePm[4][4];

	protected:		
		ROBOT_BASE *pRobot;

		const double * const pBasePrtPm{ *_BasePrtPm };
		double * const pBasePm{ *_BasePm };

		double * const pEE;
		double * const vEE;
		double * const aEE;
		double * const pIn;
		double * const vIn;
		double * const aIn;

		double &x{ pEE[0] }, &y{ pEE[1] }, &z{ pEE[2] };
		double &vx{ vEE[0] }, &vy{ vEE[1] }, &vz{ vEE[2] };
		double &ax{ aEE[0] }, &ay{ aEE[1] }, &az{ aEE[2] };
		double &l1{ pIn[0] }, &l2{ pIn[1] }, &l3{ pIn[2] };
		double &vl1{ vIn[0] }, &vl2{ vIn[1] }, &vl3{ vIn[2] };
		double &al1{ aIn[0] }, &al2{ aIn[1] }, &al3{ aIn[2] };

	protected:
		virtual void calculate_from_pEE(){};
		virtual void calculate_from_pIn(){};
		virtual void calculate_from_vEE(){};
		virtual void calculate_from_vIn(){};
		virtual void calculate_from_aEE(){};
		virtual void calculate_from_aIn(){};

	protected:
		LEG_BASE(ROBOT_BASE* pRobot, unsigned beginPos);
		virtual ~LEG_BASE() = default;

	public:
		void SetPee(const double *pEE, const char *RelativeCoodinate = "G");
		void SetVee(const double *aEE, const char *RelativeCoodinate = "G");
		void SetAee(const double *vEE, const char *RelativeCoodinate = "G");
		void SetPin(const double *pIn);
		void SetVin(const double *vIn);
		void SetAin(const double *aIn);
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		void GetVee(double *aEE, const char *RelativeCoodinate = "G") const;
		void GetAee(double *vEE, const char *RelativeCoodinate = "G") const;
		void GetPin(double *pIn) const;
		void GetVin(double *vIn) const;
		void GetAin(double *aIn) const;
		/*!
		* \brief 获取单腿坐标系下的正向力雅克比矩阵（已知输入求输出）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetFceJacDir(double *jac, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的拟向力雅克比矩阵（已知输出求输入）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetFceJacInv(double *jac, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的正向速度雅克比矩阵（已知输入求输出）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetVelJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的拟向速度雅克比矩阵（已知输出求输入）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetVelJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的正向加速度雅克比矩阵（已知输入求输出）。
		* \param jac 3乘3的雅克比矩阵。
		* \param c 残余项。
		* \param RelativeCoodinate 输出加速度所在的坐标系。
		* 输入输出关系可以用下式计算：
		* Aee=jac*Ain+c
		*/
		void GetAccJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的拟向加速度雅克比矩阵（已知输出求输入）。
		* \param jac 3乘3的雅克比矩阵。
		* \param c 残余项。
		* \param RelativeCoodinate 输出加速度所在的坐标系。
		* 输入输出关系可以用下式计算：
		* Ain=jac*Aee+c
		*/
		void GetAccJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
	};

	class ROBOT_BASE
	{
		friend class LEG_BASE;

	private:
		double _BodyPm[4][4], _BodyVel[6], _BodyAcc[6];

	protected:
		LEG_BASE *pLegs[6];

	protected:
		double pEE[18];
		double vEE[18];
		double aEE[18];
		double pIn[18];
		double vIn[18];
		double aIn[18];

		double *const pBodyPm{ *_BodyPm };
		double *const pBodyVel{ _BodyVel };
		double *const pBodyAcc{ _BodyAcc };
		

	public:
		ROBOT_BASE();
		virtual ~ROBOT_BASE() = default;


		void GetBodyPm(double *bodypm) const;
		void GetBodyVel(double *bodyvel) const;
		void GetBodyAcc(double *bodyacc) const;

		void SetPee(const double *pEE = nullptr, const double *bodyep = nullptr, const char *RelativeCoodinate = "G");
		void SetVee(const double *aEE = nullptr, const double *bodyep = nullptr, const char *RelativeCoodinate = "G");
		void SetAee(const double *vEE = nullptr, const double *bodyep = nullptr, const char *RelativeCoodinate = "G");
		void SetPin(const double *pIn = nullptr, const double *bodyep = nullptr);
		void SetVin(const double *vIn = nullptr, const double *bodyep = nullptr);
		void SetAin(const double *aIn = nullptr, const double *bodyep = nullptr);
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		void GetVee(double *aEE, const char *RelativeCoodinate = "G") const;
		void GetAee(double *vEE, const char *RelativeCoodinate = "G") const;
		void GetPin(double *pIn) const;
		void GetVin(double *vIn) const;
		void GetAin(double *aIn) const;
	};
}

#endif