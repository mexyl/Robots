#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#endif


#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

#include "Robot_Base.h"

using namespace Aris::DynKer;
using namespace std;

namespace Robots
{
	const unsigned ee_size{ 3 * sizeof(double) };
	const unsigned in_size{ 3 * sizeof(double) };
	const unsigned pm_size{ 16 * sizeof(double) };
	const unsigned vec6_size{ 6 * sizeof(double) };
	
	LEG_BASE::LEG_BASE(ROBOT_BASE* pRobot, unsigned beginPos)
		: pRobot(pRobot)
		, pEE(&pRobot->pEE[beginPos])
		, vEE(&pRobot->vEE[beginPos])
		, aEE(&pRobot->aEE[beginPos])
		, pIn(&pRobot->pIn[beginPos])
		, vIn(&pRobot->vIn[beginPos])
		, aIn(&pRobot->aIn[beginPos])
	{
	}
	
	void LEG_BASE::SetPee(const double *pEE, const char *RelativeCoodinate)
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(this->pEE, pEE, ee_size);
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_pnt(pBasePrtPm, pEE, this->pEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pm_dot_pm(pRobot->pBodyPm, pBasePrtPm, pBasePm);
			s_inv_pm_dot_pnt(pBasePm, pEE, this->pEE);
			break;
		}

		calculate_from_pEE();
		calculate_jac();
	}
	void LEG_BASE::SetPin(const double *pIn)
	{
		memcpy(this->pIn, pIn, in_size);
		calculate_from_pIn();
		calculate_jac();
	}
	void LEG_BASE::SetVee(const double *vEE, const char *RelativeCoodinate)
	{
		double pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(this->vEE, vEE, ee_size);
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
		memcpy(this->vIn, vIn, in_size);

		calculate_from_vIn();
		calculate_jac_c();
	}
	void LEG_BASE::SetAee(const double *aEE, const char *RelativeCoodinate)
	{
		double pv[3], pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(this->aEE, aEE, ee_size);
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
		memcpy(this->aIn, aIn, in_size);
		calculate_from_aIn();
	}
	void LEG_BASE::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(pEE, this->pEE, ee_size);
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
		memcpy(pIn, this->pIn, in_size);
	}
	void LEG_BASE::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(vEE, this->vEE, ee_size);
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
		memcpy(vIn, this->vIn, in_size);
	}
	void LEG_BASE::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(aEE, this->aEE, ee_size);
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
		memcpy(aIn, this->aIn, in_size);
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

	ROBOT_BASE::ROBOT_BASE()
//		:
	{
	}
	void ROBOT_BASE::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		unsigned i = 0;
		for (auto &pLeg : pLegs)
		{
			pLeg->GetPee(pEE + i, RelativeCoodinate);
			i += 3;
		}
	}
	void ROBOT_BASE::GetPin(double *pIn) const
	{
		memcpy(pIn, this->pIn, in_size * 6);
	}
	void ROBOT_BASE::GetBodyPm(double *bodypm) const
	{
		memcpy(bodypm, pBodyPm, pm_size);
	}
	void ROBOT_BASE::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		unsigned i = 0;
		for (auto &pLeg : pLegs)
		{
			pLeg->GetVee(vEE + i, RelativeCoodinate);
			i += 3;
		}
	}
	void ROBOT_BASE::GetVin(double *vIn) const
	{
		memcpy(vIn, this->vIn, in_size * 6);
	}
	void ROBOT_BASE::GetBodyVel(double *bodyvel) const
	{
		memcpy(bodyvel, this->pBodyVel, vec6_size);
	}
	void ROBOT_BASE::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		unsigned i = 0;
		for (auto &pLeg : pLegs)
		{
			pLeg->GetAee(aEE + i, RelativeCoodinate);
			i += 3;
		}
	}
	void ROBOT_BASE::GetAin(double *aIn) const
	{
		memcpy(aIn, this->aIn, in_size * 6);
	}
	void ROBOT_BASE::GetBodyAcc(double *bodyacc) const
	{
		memcpy(bodyacc, this->pBodyAcc, vec6_size);
	}
	void ROBOT_BASE::SetPee(const double *pEE, const double *bodyep, const char *RelativeCoodinate)
	{
		if (bodyep != nullptr)
		{
			s_ep2pm(bodyep, pBodyPm);
		}

		if (pEE != nullptr)
		{
			unsigned i = 0;
			for (auto &pLeg : pLegs)
			{
				pLeg->SetPee(pEE + i, RelativeCoodinate);
				i += 3;
			}
		}
	}
	void ROBOT_BASE::SetPin(const double *pIn, const double *bodyep)
	{
		if (bodyep != nullptr)
		{
			s_ep2pm(bodyep, pBodyPm);
		}

		if (pIn != nullptr)
		{
			unsigned i = 0;
			for (auto &pLeg : pLegs)
			{
				pLeg->SetPin(pIn + i);
				i += 3;
			}
		}
	}
	void ROBOT_BASE::SetVee(const double *vEE, const double *bodyvel, const char *RelativeCoodinate)
	{
		if (bodyvel != nullptr)
		{
			memcpy(pBodyVel, bodyvel, vec6_size);
		}

		if (vEE != nullptr)
		{
			unsigned i = 0;
			for (auto &pLeg : pLegs)
			{
				pLeg->SetVee(vEE + i, RelativeCoodinate);
				i += 3;
			}
		}
	}
	void ROBOT_BASE::SetVin(const double *vIn, const double *bodyvel)
	{
		if (bodyvel != nullptr)
		{
			memcpy(pBodyVel, bodyvel, vec6_size);
		}
		if (vIn != nullptr)
		{
			unsigned i = 0;
			for (auto &pLeg : pLegs)
			{
				pLeg->SetVin(vIn + i);
				i += 3;
			}
		}
	}
	void ROBOT_BASE::SetAee(const double *aEE, const double *bodyacc, const char *RelativeCoodinate)
	{
		if (bodyacc != nullptr)
		{
			memcpy(pBodyAcc, bodyacc, vec6_size);
		}
		if (aEE != nullptr)
		{
			unsigned i = 0;
			for (auto &pLeg : pLegs)
			{
				pLeg->SetAee(aEE + i, RelativeCoodinate);
				i += 3;
			}
		}
	}
	void ROBOT_BASE::SetAin(const double *aIn, const double *bodyacc)
	{
		if (bodyacc != nullptr)
		{
			memcpy(pBodyAcc, bodyacc, vec6_size);
		}
		if (aIn != nullptr)
		{
			unsigned i = 0;
			for (auto &pLeg : pLegs)
			{
				pLeg->SetAin(aIn + i);
				i += 3;
			}
		}
	}
}