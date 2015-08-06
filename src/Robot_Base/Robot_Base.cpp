#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif


#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

#include <Aris_DynKer.h>

#include "Robot_Base.h"

using namespace Aris::DynKer;
using namespace std;

namespace Robots
{
	LEG_BASE::LEG_BASE(ROBOT_BASE* pRobot)
		: pRobot(pRobot)
	{
	}
	
	void LEG_BASE::SetPee(const double *pEE, const char *RelativeCoodinate)
	{
		s_pm_dot_pm(pRobot->pBodyPm, pBasePrtPm, pBasePm);

		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(pEE, 3, this->pEE);
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_pnt(pBasePrtPm, pEE, this->pEE);
			break;
		case 'G':
		case 'O':
		default:
			s_inv_pm_dot_pnt(pBasePm, pEE, this->pEE);
			break;
		}

		calculate_from_pEE();
		calculate_jac();
	}
	void LEG_BASE::SetPin(const double *pIn)
	{
		std::copy_n(pIn, 3, this->pIn);
		calculate_from_pIn();
		calculate_jac();
	}
	void LEG_BASE::SetVee(const double *vEE, const char *RelativeCoodinate)
	{
		double pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(vEE, 3, this->vEE);
			break;
		case 'B':
		case 'M':
			s_dgemmTN(3, 1, 3, 1, pBasePrtPm, 4, vEE, 1, 0, this->vEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			GetPee(pnt, "G");
			s_inv_pv2pv(pBasePm, pRobot->pBodyVel, pnt, vEE, this->vEE);
			break;
		}

		calculate_from_vEE();
		calculate_jac_c();
	}
	void LEG_BASE::SetVin(const double *vIn)
	{
		std::copy_n(vIn, 3, this->vIn);

		calculate_from_vIn();
		calculate_jac_c();
	}
	void LEG_BASE::SetAee(const double *aEE, const char *RelativeCoodinate)
	{
		double pv[3], pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(aEE, 3, this->aEE);
			break;
		case 'B':
		case 'M':
			s_dgemmTN(3, 1, 3, 1, pBasePrtPm, 4, aEE, 1, 0, this->aEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			GetPee(pnt, "G");
			GetVee(pv, "G");
			s_inv_pa2pa(pBasePm, pRobot->pBodyVel, pRobot->pBodyAcc, pnt, pv, aEE, this->aEE);
			break;
		}

		calculate_from_aEE();
	}
	void LEG_BASE::SetAin(const double *aIn)
	{
		std::copy_n(aIn, 3, this->aIn);
		calculate_from_aIn();
	}
	void LEG_BASE::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->pEE, 3, pEE);
			break;
		case 'B':
		case 'M':
			s_pm_dot_pnt(pBasePrtPm, this->pEE, pEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pm_dot_pnt(pBasePm, this->pEE, pEE);
			break;
		}
	}
	void LEG_BASE::GetPin(double *pIn) const
	{
		std::copy_n(this->pIn, 3, pIn);
	}
	void LEG_BASE::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->vEE, 3, vEE);
			break;
		case 'B':
		case 'M':
			s_dgemm(3, 1, 3, 1, pBasePrtPm, 4, this->vEE, 1, 0, vEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			s_pv2pv(pBasePm, pRobot->pBodyVel, this->pEE, this->vEE, vEE);
			break;
		}
	}
	void LEG_BASE::GetVin(double *vIn) const
	{
		std::copy_n(this->vIn, 3, vIn);
	}
	void LEG_BASE::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			std::copy_n(this->aEE, 3, aEE);
			break;
		case 'B':
		case 'M':
			s_pa2pa(pBasePrtPm, 0, 0, this->pEE, this->vEE, this->aEE, aEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pa2pa(pBasePm, pRobot->pBodyVel, pRobot->pBodyAcc, this->pEE, this->vEE, this->aEE, aEE);
			break;
		}
	}
	void LEG_BASE::GetAin(double *aIn) const
	{
		std::copy_n(this->aIn, 3, aIn);
	}

	void LEG_BASE::GetFceJacDir(double *jac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetVelJacInv(*locJac, 0, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LEG_BASE::GetFceJacInv(double *jac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		double locJac[3][3];

		GetVelJacDir(*locJac, 0, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LEG_BASE::GetVelJacDir(double *jac, double *c, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *_jac_vel_dir, sizeof(_jac_vel_dir));
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double) * 3);
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBasePrtPm, 4, *_jac_vel_dir, 3, 0, jac, 3);
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double) * 3);
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBasePm, 4, *_jac_vel_dir, 3, 0, jac, 3);
			}
			if (c != nullptr)
			{
				s_pv2pv(pBasePm, pRobot->pBodyVel, this->pEE, 0, c);
			}
			break;
		}

	}
	void LEG_BASE::GetVelJacInv(double *jac, double *c, const char *RelativeCoodinate) const
	{
		double tem[3];
		
		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *_jac_vel_inv, sizeof(_jac_vel_inv));
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double) * 3);
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *_jac_vel_inv, 3, pBasePrtPm, 4, 0, jac, 3);
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double) * 3);
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *_jac_vel_inv, 3, pBasePm, 4, 0, jac, 3);
			}
			if (c != nullptr)
			{
				s_pv2pv(pBasePm, pRobot->pBodyVel, this->pEE, 0, c);
				s_dgemmTN(3, 1, 3, 1, pBasePm, 4, c, 1, 0, tem, 1);
				s_dgemm(3, 1, 3, -1, *_jac_vel_inv, 3, tem, 1, 0, c, 1);
			}
			break;
		}
	}
	void LEG_BASE::GetAccJacDir(double *jac, double *c, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *_jac_vel_dir, sizeof(_jac_vel_dir));
			}
			if (c != 0)
			{
				memcpy(c, _c_acc_dir, sizeof(_c_acc_dir));
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBasePrtPm, 4, *_jac_vel_dir, 3, 0, jac, 3);
			}
			if (c != nullptr)
			{
				s_dgemm(3, 1, 3, 1, pBasePrtPm, 4, _c_acc_dir, 1, 0, c, 1);
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBasePm, 4, *_jac_vel_dir, 3, 0, jac, 3);
			}

			if (c != 0)
			{
				s_pa2pa(pBasePm, pRobot->pBodyVel, pRobot->pBodyAcc, this->pEE, this->vEE, _c_acc_dir, c);
			}
			break;
		}
	}
	void LEG_BASE::GetAccJacInv(double *jac, double *c, const char *RelativeCoodinate) const
	{
		double tem1[3], tem2[3], tem3[3];
		
		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *_jac_vel_inv, sizeof(_jac_vel_inv));
			}
			if (c != nullptr)
			{
				memcpy(c, _c_acc_inv, sizeof(_c_acc_inv));
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *_jac_vel_inv, 3, pBasePrtPm, 4, 0, jac, 3);
			}
			if (c != nullptr)
			{
				memcpy(c, _c_acc_inv, sizeof(_c_acc_inv));
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *_jac_vel_inv, 3, pBasePm, 4, 0, jac, 3);
			}

			if (c != nullptr)
			{
				memcpy(c, _c_acc_inv, sizeof(_c_acc_inv));
				this->GetPee(tem1, "G");
				this->GetVee(tem2, "G");
				s_inv_pa2pa(pBasePm, pRobot->pBodyVel, pRobot->pBodyAcc, tem1, tem2, 0, tem3);
				s_dgemm(3, 1, 3, 1, *_jac_vel_inv, 3, tem3, 1, 1, c, 1);
			}
			break;
		}
	}

	void ROBOT_BASE::GetBodyPm(double *bodypm) const
	{ 
		std::copy_n(pBodyPm, 16, bodypm);
	};
	void ROBOT_BASE::GetBodyPe(double *bodype, const char *eurType) const
	{ 
		s_pm2pe(pBodyPm, bodype, eurType); 
	};
	void ROBOT_BASE::GetBodyVel(double *bodyvel) const
	{ 
		std::copy_n(this->pBodyVel, 6, bodyvel);
	};
	void ROBOT_BASE::GetBodyAcc(double *bodyacc) const
	{ 
		std::copy_n(this->pBodyAcc, 6, bodyacc);
	};
	void ROBOT_BASE::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPee(pEE + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::GetPin(double *pIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPin(pIn + i * 3);
		}
	}
	void ROBOT_BASE::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVee(vEE + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::GetVin(double *vIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetVin(vIn + i * 3);
		}
	}
	void ROBOT_BASE::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAee(aEE + i * 3, RelativeCoodinate);
		}
	}
	void ROBOT_BASE::GetAin(double *aIn) const
	{
		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetAin(aIn + i*3);
		}
	}
	void ROBOT_BASE::SetPee(const double *pEE, const double *bodyep, const char *RelativeCoodinate, const char *eurType)
	{
		if (bodyep)
		{
			s_pe2pm(bodyep, pBodyPm, eurType);
		}

		if (pEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetPee(pEE + i * 3, RelativeCoodinate);
			}
		}
	}
	void ROBOT_BASE::SetPin(const double *pIn, const double *bodyep, const char *eurType)
	{
		if (bodyep)
		{
			s_pe2pm(bodyep, pBodyPm, eurType);
		}

		if (pIn)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetPin(pIn + i * 3);
			}
		}
	}
	void ROBOT_BASE::SetVee(const double *vEE, const double *bodyvel, const char *RelativeCoodinate)
	{
		if (bodyvel)
		{
			std::copy_n(bodyvel, 6, pBodyVel);
		}

		if (vEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetVee(vEE + i * 3, RelativeCoodinate);
			}
		}
	}
	void ROBOT_BASE::SetVin(const double *vIn, const double *bodyvel)
	{
		if (bodyvel)
		{
			std::copy_n(bodyvel, 6, pBodyVel);
		}
		if (vIn)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetVin(vIn + i * 3);
			}
		}
	}
	void ROBOT_BASE::SetAee(const double *aEE, const double *bodyacc, const char *RelativeCoodinate)
	{
		if (bodyacc)
		{
			std::copy_n(bodyacc, 6, pBodyAcc);
		}
		if (aEE)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetAee(aEE + i * 3, RelativeCoodinate);
			}
		}
	}
	void ROBOT_BASE::SetAin(const double *aIn, const double *bodyacc)
	{
		if (bodyacc)
		{
			std::copy_n(bodyacc, 6, pBodyAcc);
		}
		if (aIn)//= if(aIn!=nullptr)
		{
			for (int i = 0; i < 6; i++)
			{
				pLegs[i]->SetAin(aIn + i * 3);
			}
		}
	}




}