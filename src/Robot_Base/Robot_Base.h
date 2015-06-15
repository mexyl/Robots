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

		double _c_acc_dir[3];
		double _c_acc_inv[3];

		double _jac_vel_dir[3][3];
		double _jac_vel_inv[3][3];

	protected:
		virtual void calculate_from_pEE(){};
		virtual void calculate_from_pIn(){};
		virtual void calculate_from_vEE(){};
		virtual void calculate_from_vIn(){};
		virtual void calculate_from_aEE(){};
		virtual void calculate_from_aIn(){};

		virtual void calculate_jac(){};
		virtual void calculate_jac_c(){};

	protected:
		LEG_BASE(ROBOT_BASE* pRobot, unsigned beginPos);
		virtual ~LEG_BASE() = default;

	public:
		/*!
		* \brief Set the position of end-effector
		* \param pEE position of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetPee(const double *pEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Set the velocity of end-effector, must be called after position is set
		* \param vEE velocity of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetVee(const double *vEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Set the velocity of end-effector, must be called after velocity is set
		* \param vEE acceleration of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetAee(const double *aEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Set the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void SetPin(const double *pIn);
		/*!
		* \brief Set the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void SetVin(const double *vIn);
		/*!
		* \brief Set the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void SetAin(const double *aIn);
		/*!
		* \brief Get the position of end-effector
		* \param pEE position of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Get the velocity of end-effector
		* \param vEE velocity of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetVee(double *vEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Get the acceleration of end-effector
		* \param pEE acceleration of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetAee(double *aEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Get the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void GetPin(double *pIn) const;
		/*!
		* \brief Get the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void GetVin(double *vIn) const;
		/*!
		* \brief Get the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void GetAin(double *aIn) const;


		void GetFceJacDir(double *jac, const char *RelativeCoodinate = "G") const;
		void GetFceJacInv(double *jac, const char *RelativeCoodinate = "G") const;
		void GetVelJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		void GetVelJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		void GetAccJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
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

		/*!
		* \brief Get the body pose matrix
		* \param body pose matrix, double array with 16 elements
		*/
		void GetBodyPm(double *bodypm) const;
		/*!
		* \brief Get the body velocity
		* \param body velocity, double array with 6 elements
		*/
		void GetBodyVel(double *bodyvel) const;
		/*!
		* \brief Get the body acceleration
		* \param body acceleration, double array with 6 elements
		*/
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