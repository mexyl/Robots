#ifndef ROBOT_EXPORTS
#define ROBOT_EXPORTS
#endif

#include "Platform.h"

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "Hexapod_Robot.h"
#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

using namespace Aris::DynKer;
using namespace std;

namespace Hexapod_Robot
{	
	double acc_even(unsigned int n, unsigned int i)
	{
		return 1.0 / n / n  * i * i;
	}

	double dec_even(unsigned int n, unsigned int i)
	{
		return 1.0 - 1.0 / n / n * (n-i)*(n-i);
	}

	double even(unsigned int n, unsigned int i)
	{
		return 1.0 / n*i;
	}
	
	LEG::LEG(const char *Name, ROBOT* probot)
		: Aris::DynKer::OBJECT(nullptr,Name)
		, pRobot(probot), U2x(0), U2y(0), U2z(0), U3x(0), U3y(0), U3z(0), S2x(0), S2y(0), S2z(0), S3x(0)
		, S3y(0), S3z(0), Sfx(0), Sfy(0), Sfz(0), D1(0), H1(0), D2(0), H2(0)
	{}
	LEG::~LEG()
	{
	}
	
	void LEG::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(pEE, this->pEE, sizeof(this->pEE));
			break;
		case 'B':
		case 'M':
			s_pm_dot_pnt(pBase->GetPrtPmPtr(), this->pEE, pEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pm_dot_pnt(pBase->GetPmPtr(), this->pEE, pEE);
			break;
		}
	}
	void LEG::GetPcd(double *pCd) const
	{
		memcpy(pCd, this->pCD, sizeof(this->pCD));
	}
	void LEG::GetPin(double *pIn) const
	{
		memcpy(pIn, this->pIn, sizeof(this->pIn));
	}
	void LEG::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(vEE, this->vEE, sizeof(this->vEE));
			break;
		case 'B':
		case 'M':
			s_dgemm(3, 1, 3, 1, pBase->GetPrtPmPtr(), 4, this->vEE, 1, 0, vEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			s_pv2pv(pBase->GetPmPtr(), pBase->GetVelPtr(), this->pEE, this->vEE, vEE);
			break;
		}
	}
	void LEG::GetVcd(double *vCd) const
	{
		memcpy(vCd, this->vCD,sizeof(this->vCD));
	}
	void LEG::GetVin(double *vIn) const
	{
		memcpy(vIn, this->vIn, sizeof(this->vIn));
	}
	void LEG::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(aEE, this->aEE, sizeof(this->aEE));
			break;
		case 'B':
		case 'M':
			s_pa2pa(pBase->GetPrtPmPtr(), 0, 0, this->pEE, this->vEE, this->aEE, aEE);
			break;
		case 'G':
		case 'O':
		default:
			s_pa2pa(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), this->pEE, this->vEE, this->aEE, aEE);
			break;
		}
	}
	void LEG::GetAcd(double *aCD) const
	{
		memcpy(aCD, this->aCD, sizeof(this->aCD));
	}
	void LEG::GetAin(double *aIn) const
	{
		memcpy(aIn, this->aIn, sizeof(this->aIn));
	}
	
	void LEG::GetFceJacDir(double *jac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		static double locJac[3][3];

		GetVelJacInv(*locJac, 0, RelativeCoodinate);
		s_transpose(3,3,*locJac,3,jac,3);
	}
	void LEG::GetFceJacInv(double *jac, const char *RelativeCoodinate) const
	{
		/*力雅克比是速度雅克比转置的逆*/
		static double locJac[3][3];

		GetVelJacDir(*locJac, 0, RelativeCoodinate);
		s_transpose(3, 3, *locJac, 3, jac, 3);
	}
	void LEG::GetVelJacDir(double *jac, double *c, const char *RelativeCoodinate) const
	{
		
		static int ipiv[3];
		static double locJ2[3][3], legJac[3][3];

		memcpy(*legJac, *this->J1, sizeof(this->J1));
		memcpy(*locJ2, *this->J2, sizeof(this->J2));
		s_dgesvT(3, 3, *locJ2, 3, ipiv, *legJac, 3);

		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *legJac, sizeof(legJac));
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double)* 3);
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBase->GetPrtPmPtr(), 4, *legJac, 3, 0, jac, 3);
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double)* 3);
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBase->GetPmPtr(), 4, *legJac, 3, 0, jac, 3);
			}
			if (c != nullptr)
			{
				s_pv2pv(pBase->GetPmPtr(), pBase->GetVelPtr(), this->pEE, 0, c);
			}
			break;
		}
		
	}
	void LEG::GetVelJacInv(double *jac, double *c, const char *RelativeCoodinate) const
	{
		static int ipiv[3];
		static double locJ1[3][3], legJac[3][3],tem[3];

		memcpy(*legJac, *this->J2, sizeof(this->J2));
		memcpy(*locJ1, *this->J1, sizeof(this->J1));
		s_dgesvT(3, 3, *locJ1, 3, ipiv, *legJac, 3);

		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *legJac, sizeof(legJac));
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double)* 3);
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *legJac, 3, pBase->GetPrtPmPtr(), 4, 0, jac, 3);
			}
			if (c != nullptr)
			{
				memset(c, 0, sizeof(double)* 3);
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *legJac, 3, pBase->GetPmPtr(), 4, 0, jac, 3);
			}
			if (c != nullptr)
			{
				s_pv2pv(pBase->GetPmPtr(), pBase->GetVelPtr(), this->pEE, 0, c);
				s_dgemmTN(3, 1, 3, 1, pBase->GetPmPtr(), 4, c, 1, 0, tem, 1);
				s_dgemm(3, 1, 3, -1, *legJac, 3, tem, 1, 0, c, 1);
			}
			break;
		}
	}
	void LEG::GetAccJacDir(double *jac, double *c, const char *RelativeCoodinate) const
	{
		static int ipiv[3];
		static double inv_J2[3][3], legJac[3][3], vJac[3][3], tem1[3], tem2[3];


		memcpy(*inv_J2, *this->J2, sizeof(this->J2));
		s_dgeinv(3, *inv_J2, 3, ipiv);
		s_dgemm(3, 3, 3, 1, *J1, 3, *inv_J2, 3, 0, *legJac, 3);
		
		s_dgemm(3, 1, 3, 1, *vJ2, 3, this->vCD, 1, 0, tem1, 1);
		s_dgemm(3, 1, 3, -1, *legJac, 3, tem1, 1, 0, tem2, 1);
		s_dgemm(3, 1, 3, 1, *vJ1, 3, this->vCD, 1, 1, tem2, 1);

		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *legJac, sizeof(legJac));
			}
			if (c != 0)
			{
				memcpy(c, tem2, sizeof(tem2));
			}	
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBase->GetPrtPmPtr(), 4, *legJac, 3, 0, jac, 3);
			}		
			if (c != nullptr)
			{
				s_dgemm(3, 1, 3, 1, pBase->GetPrtPmPtr(), 4, tem2, 1, 0, c, 1);
			}	
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemm(3, 3, 3, 1, pBase->GetPmPtr(), 4, *legJac, 3, 0, jac, 3);
			}
			
			if (c != 0)
			{
				s_pa2pa(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), this->pEE, this->vEE, tem2, c);
			}
			break;
		}
	}
	void LEG::GetAccJacInv(double *jac, double *c, const char *RelativeCoodinate) const
	{
		static int ipiv[3];
		static double inv_J1[3][3], legJac[3][3], vJac[3][3], tem1[3], tem2[3],tem3[3];

		memcpy(*inv_J1, *this->J1, sizeof(this->J1));
		s_dgeinv(3, *inv_J1, 3, ipiv);
		s_dgemm(3, 3, 3, 1, *J2, 3, *inv_J1, 3, 0, *legJac, 3);

		s_dgemm(3, 1, 3, 1, *vJ1, 3, this->vCD, 1, 0, tem1, 1);
		s_dgemm(3, 1, 3, -1, *legJac, 3, tem1, 1, 0, tem2, 1);
		s_dgemm(3, 1, 3, 1, *vJ2, 3, this->vCD, 1, 1, tem2, 1);

		switch (*RelativeCoodinate)
		{
		case 'L':
			if (jac != nullptr)
			{
				memcpy(jac, *legJac, sizeof(legJac));
			}
			if (c != 0)
			{
				memcpy(c, tem2, sizeof(tem2));
			}
			break;
		case 'M':
		case 'B':
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *legJac, 3, pBase->GetPrtPmPtr(), 4, 0, jac, 3);
			}
			if (c != nullptr)
			{
				memcpy(c, tem2, sizeof(tem2));
			}
			break;
		case 'G':
		case 'O':
		default:
			if (jac != nullptr)
			{
				s_dgemmNT(3, 3, 3, 1, *legJac, 3, pBase->GetPmPtr(), 4, 0, jac, 3);
			}

			if (c != 0)
			{
				memcpy(c, tem2, sizeof(tem2));
				this->GetPee(tem1, "G");
				this->GetVee(tem2, "G");
				s_inv_pa2pa(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), tem1, tem2, 0, tem3);
				s_dgemm(3, 1, 3, 1, *legJac, 3, tem3, 1, 1, c, 1);
			}
			break;
		}
	}

	void LEG::SetPee(const double *pEE, const char *RelativeCoodinate)
	{
		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(this->pEE, pEE, sizeof(this->pEE));
			break;
		case 'B':
		case 'M':
			s_inv_pm_dot_pnt(pBase->GetPrtPmPtr(), pEE, this->pEE);
			break;
		case 'G':
		case 'O':
		default:
			pBase->Update();
			s_inv_pm_dot_pnt(pBase->GetPmPtr(), pEE, this->pEE);
			break;
		}
		_CalCdByPos();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LEG::SetPcd(const double *pCD)
	{
		memcpy(this->pCD, pCD, sizeof(this->pCD));
		_CalVarByCd();
		_CalPartByVar();
	}
	void LEG::SetPin(const double *pIn)
	{
		memcpy(this->pIn, pIn, sizeof(this->pIn));
		_CalCdByPlen();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LEG::SetPin2(const double *pIn)
	{
		memcpy(this->pIn, pIn, sizeof(this->pIn));
		_CalCdByPlen2();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LEG::SetVee(const double *vEE, const char *RelativeCoodinate)
	{
		double pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(this->vEE, vEE, sizeof(this->vEE));
			break;
		case 'B':
		case 'M':
			s_dgemmTN(3, 1, 3, 1, pBase->GetPrtPmPtr(), 4, vEE, 1, 0, this->vEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			GetPee(pnt, "G");
			s_inv_pv2pv(pBase->GetPmPtr(), pBase->GetVelPtr(), pnt, vEE, this->vEE);
			break;
		}

		_CalVcdByVpos();
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LEG::SetVcd(const double *vcd)
	{
		memcpy(this->vCD, vcd, sizeof(this->vCD));
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LEG::SetVin(const double *vIn)
	{
		memcpy(this->vIn, vIn, sizeof(this->vIn));
		_CalVcdByVplen();
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LEG::SetAee(const double *aEE, const char *RelativeCoodinate)
	{
		static double pv[3], pnt[3];

		switch (*RelativeCoodinate)
		{
		case 'L':
			memcpy(this->aEE, aEE, sizeof(this->aEE));
			break;
		case 'B':
		case 'M':
			s_dgemmTN(3, 1, 3, 1, pBase->GetPrtPmPtr(), 4, aEE, 1, 0, this->aEE, 1);
			break;
		case 'G':
		case 'O':
		default:
			GetPee(pnt, "G");
			GetVee(pv, "G");
			s_inv_pa2pa(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), pnt, pv, aEE, this->aEE);
			break;
		}

		_CalAcdByApos();
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LEG::SetAcd(const double *aCD)
	{
		memcpy(this->aCD, aCD, sizeof(this->aCD));
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LEG::SetAin(const double *aIn)
	{
		memcpy(this->aIn, aIn, sizeof(this->aIn));
		_CalAcdByAplen();
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LEG::FastDynMtxInPrt()
	{
		static double Loc_C[36][36], Loc_a_c[36];
		static int ipiv[36];

		static double s[36];
		double rcond = 0.0000001;

		static double tem[6][6], tem_vel[6], vcro[6][6], tem_cdot[6][6], tem_cdot2[6][6];


		/*初始化*/
		memset(*_C, 0, 36 * 36 * sizeof(double));
		memset(*_I, 0, 36 * 36 * sizeof(double));
		memset(_f, 0, 36 * sizeof(double));
		memset(_v, 0, 36 * sizeof(double));
		memset(_a_c, 0, 36 * sizeof(double));

		memset(_a, 0, 36 * sizeof(double));
		memset(_epsilon, 0, 36 * sizeof(double));

		memset(_c_M, 0, 36 * 4 * sizeof(double));
		
		/*计算C*/
		/*
		          U1    U2    U3     P1      P2       P3      S2     S3     M1       M2       M3
		P1a      0*0                -0*12                                  -0*33
		P2a            6*4                  -6*17                                  -6*34
		P3a                  12*8                  -12*22                                  -12*35
		Thigh                      18*12                    18*27   18*30  18*33
		P2b                                24*17           -24*27                   24*34
		P3b                                         30*22          -30*30                   30*35
		*/

		/*更新每个部件*/
		pRobot->pBody->UpdateInPrt();

		for (auto &i : pPrts)
		{
			i->UpdateInPrt();
		}

		/*更新每个关节和驱动*/
		for (auto &i : pJnts)
		{
			i->UpdateInPrt();
		}
		for (auto &i : pMots)
		{
			i->UpdateInPrt();
		}

		/*复制约束矩阵*/
		s_block_cpy(6, 4, pU1->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 0, 0, 36);
		s_block_cpy(6, 4, pU2->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 6, 4, 36);
		s_block_cpy(6, 4, pU3->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 12, 8, 36);
		s_block_cpy(6, 5, pP1->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 18, 12, 36);
		s_block_cpy(6, 5, pP1->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 0, 12, 36);
		s_block_cpy(6, 5, pP2->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 24, 17, 36);
		s_block_cpy(6, 5, pP2->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 6, 17, 36);
		s_block_cpy(6, 5, pP3->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 30, 22, 36);
		s_block_cpy(6, 5, pP3->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 12, 22, 36);
		s_block_cpy(6, 3, pS2->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 18, 27, 36);
		s_block_cpy(6, 3, pS2->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 24, 27, 36);
		s_block_cpy(6, 3, pS3->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 18, 30, 36);
		s_block_cpy(6, 3, pS3->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 30, 30, 36);

		if (pSf->GetActive())
		{
			/*若该腿支撑，则使用Sf副约束*/
			pSf->UpdateInPrt();
			s_block_cpy(6, 3, pSf->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 18, 33, 36);

			/*更新驱动矩阵M*/
			s_block_cpy(6, 1, pM1->GetPrtCstMtxIPtr(), 0, 0, 6, *_c_M, 18, 1, 4);
			s_block_cpy(6, 1, pM1->GetPrtCstMtxJPtr(), 0, 0, 6, *_c_M, 0, 1, 4);
			s_block_cpy(6, 1, pM2->GetPrtCstMtxIPtr(), 0, 0, 6, *_c_M, 24, 2, 4);
			s_block_cpy(6, 1, pM2->GetPrtCstMtxJPtr(), 0, 0, 6, *_c_M, 6, 2, 4);
			s_block_cpy(6, 1, pM3->GetPrtCstMtxIPtr(), 0, 0, 6, *_c_M, 30, 3, 4);
			s_block_cpy(6, 1, pM3->GetPrtCstMtxJPtr(), 0, 0, 6, *_c_M, 12, 3, 4);
		}
		else
		{
			/*否则，更新驱动的约束矩阵*/
			s_block_cpy(6, 1, pM1->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 18, 33, 36);
			s_block_cpy(6, 1, pM1->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 0, 33, 36);
			s_block_cpy(6, 1, pM2->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 24, 34, 36);
			s_block_cpy(6, 1, pM2->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 6, 34, 36);
			s_block_cpy(6, 1, pM3->GetPrtCstMtxIPtr(), 0, 0, 6, *_C, 30, 35, 36);
			s_block_cpy(6, 1, pM3->GetPrtCstMtxJPtr(), 0, 0, 6, *_C, 12, 35, 36);
		}

		/*更新右侧的c_M矩阵*/
		s_daxpy(6, -1, pP1a->GetPrtFgPtr(), 1, &_c_M[0][0], 4);
		s_daxpy(6, 1, pP1a->GetPrtFvPtr(), 1, &_c_M[0][0], 4);
		s_tmv_dot_vel(pP1a->GetPrtTmvPtr(), pP1a->GetAccPtr(), pP1a->GetPrtAccPtr());
		s_dgemm(6, 1, 6, 1, pP1a->GetPrtImPtr(), 6, pP1a->GetPrtAccPtr(), 1, 1, &_c_M[0][0], 4);

		s_daxpy(6, -1, pP2a->GetPrtFgPtr(), 1, &_c_M[6][0], 4);
		s_daxpy(6, 1, pP2a->GetPrtFvPtr(), 1, &_c_M[6][0], 4);
		s_tmv_dot_vel(pP2a->GetPrtTmvPtr(), pP2a->GetAccPtr(), pP2a->GetPrtAccPtr());
		s_dgemm(6, 1, 6, 1, pP2a->GetPrtImPtr(), 6, pP2a->GetPrtAccPtr(), 1, 1, &_c_M[6][0], 4);

		s_daxpy(6, -1, pP3a->GetPrtFgPtr(), 1, &_c_M[12][0], 4);
		s_daxpy(6, 1, pP3a->GetPrtFvPtr(), 1, &_c_M[12][0], 4);
		s_tmv_dot_vel(pP3a->GetPrtTmvPtr(), pP3a->GetAccPtr(), pP3a->GetPrtAccPtr());
		s_dgemm(6, 1, 6, 1, pP3a->GetPrtImPtr(), 6, pP3a->GetPrtAccPtr(), 1, 1, &_c_M[12][0], 4);

		s_daxpy(6, -1, pThigh->GetPrtFgPtr(), 1, &_c_M[18][0], 4);
		s_daxpy(6, 1, pThigh->GetPrtFvPtr(), 1, &_c_M[18][0], 4);
		s_tmv_dot_vel(pThigh->GetPrtTmvPtr(), pThigh->GetAccPtr(), pThigh->GetPrtAccPtr());
		s_dgemm(6, 1, 6, 1, pThigh->GetPrtImPtr(), 6, pThigh->GetPrtAccPtr(), 1, 1, &_c_M[18][0], 4);

		s_daxpy(6, -1, pP2b->GetPrtFgPtr(), 1, &_c_M[24][0], 4);
		s_daxpy(6, 1, pP2b->GetPrtFvPtr(), 1, &_c_M[24][0], 4);
		s_tmv_dot_vel(pP2b->GetPrtTmvPtr(), pP2b->GetAccPtr(), pP2b->GetPrtAccPtr());
		s_dgemm(6, 1, 6, 1, pP2b->GetPrtImPtr(), 6, pP2b->GetPrtAccPtr(), 1, 1, &_c_M[24][0], 4);

		s_daxpy(6, -1, pP3b->GetPrtFgPtr(), 1, &_c_M[30][0], 4);
		s_daxpy(6, 1, pP3b->GetPrtFvPtr(), 1, &_c_M[30][0], 4);
		s_tmv_dot_vel(pP3b->GetPrtTmvPtr(), pP3b->GetAccPtr(), pP3b->GetPrtAccPtr());
		s_dgemm(6, 1, 6, 1, pP3b->GetPrtImPtr(), 6, pP3b->GetPrtAccPtr(), 1, 1, &_c_M[30][0], 4);
	}
	void LEG::FastDynEeForce(const double *fIn_in, double *fEE_out, const char *RelativeCoodinate)
	{
		static double Loc_C[36][36],Loc_c_M[36][4];
		static int ipiv[36];
		double f[3],jac[3][3];

		pSf->Deactivate();
		FastDynMtxInPrt();

		memcpy(Loc_C, _C, 36 * 36 * sizeof(double));
		memcpy(Loc_c_M, _c_M, 36 * 4 * sizeof(double));

		s_dgesv(36, 1, *Loc_C, 36, ipiv, *Loc_c_M, 4);

		f[0] = Loc_c_M[33][0] + pM1->GetFrcCoePtr()[0] * s_sgn(*pM1->GetV_mPtr())
			+ pM1->GetFrcCoePtr()[1] * (*pM1->GetV_mPtr())
			+ pM1->GetFrcCoePtr()[2] * (*pM1->GetA_mPtr());

		f[1] = Loc_c_M[34][0] + pM2->GetFrcCoePtr()[0] * s_sgn(*pM2->GetV_mPtr())
			+ pM2->GetFrcCoePtr()[1] * (*pM2->GetV_mPtr())
			+ pM2->GetFrcCoePtr()[2] * (*pM2->GetA_mPtr());

		f[2] = Loc_c_M[35][0] + pM3->GetFrcCoePtr()[0] * s_sgn(*pM3->GetV_mPtr())
			+ pM3->GetFrcCoePtr()[1] * (*pM3->GetV_mPtr())
			+ pM3->GetFrcCoePtr()[2] * (*pM3->GetA_mPtr());

		s_daxpy(3, -1, fIn_in, 1, f, 1);
		
		GetFceJacDir(*jac, RelativeCoodinate);

		s_dgemm(3, 1, 3, -1, *jac, 3, f, 1, 0, fEE_out, 1);
	}
	void LEG::Calibrate(const int n, const double *plen, const double *vplen, const double *aplen, const double *fplen)
	{
		/*
		vector<double[60]> _Calibration_m(n * 3);
		vector<double[3]> _Calibration_fce(n * 3);
		
		double cm[6][6];
		double _inv_of_C[36][36], A[3][6], B[3][6], q[6],v[6];
		int ipiv[36];

		memset(_Calibration_m.data(), 0, sizeof(double)*n * 3 * 60);
		memcpy(_Calibration_fce.data(), fplen, sizeof(double)*n * 3);
		
		this->pSf->Deactivate();

		for (int i = 0; i < n; ++i)
		{
			this->SetPin(plen + i * 3);
			this->SetVin(vplen + i * 3);
			this->SetAin(aplen + i * 3);

			this->FastDynMtxInPrt();

			memcpy(_inv_of_C, this->_C, sizeof(double)* 36 * 36);

			s_dgeinv(36,*_inv_of_C,36,ipiv);

			for (int j = 0; j < 6; j++)
			{
				PART *pPrt=pPrts[j];

				s_block_cpy(3, 6, *_inv_of_C, 33, j * 6, 36, *A, 0, 0, 6);

				memset(q, 0, sizeof(double)* 6);
				memcpy(q, pPrt->GetPrtAccPtr(), sizeof(double)* 6);
				s_daxpy(6, -1, pPrt->GetPrtGravityPtr(), 1, q, 1);
				
				s_cmf(pPrt->GetPrtVelPtr(), *cm);
				s_dgemm(3, 6, 6, 1, *A, 6, *cm, 6, 0, *B, 6);

				memcpy(v, pPrt->GetPrtVelPtr(), sizeof(double)* 6);

				for (int k = 0; k < 3; k++)
				{
					_Calibration_m[i * 3 + k][j * 10] = A[k][0] * q[0] + A[k][1] * q[1] + A[k][2] * q[2];
					_Calibration_m[i * 3 + k][j * 10 + 1] = A[k][1] * q[5] + A[k][5] * q[1] - A[k][2] * q[4] - A[k][4] * q[2];
					_Calibration_m[i * 3 + k][j * 10 + 2] = A[k][2] * q[3] + A[k][3] * q[2] - A[k][0] * q[5] - A[k][5] * q[0];
					_Calibration_m[i * 3 + k][j * 10 + 3] = A[k][0] * q[4] + A[k][4] * q[0] - A[k][1] * q[3] - A[k][3] * q[1];
					_Calibration_m[i * 3 + k][j * 10 + 4] = A[k][3] * q[3];
					_Calibration_m[i * 3 + k][j * 10 + 5] = A[k][4] * q[4];
					_Calibration_m[i * 3 + k][j * 10 + 6] = A[k][5] * q[5];
					_Calibration_m[i * 3 + k][j * 10 + 7] = A[k][3] * q[4] + A[k][4] * q[3];
					_Calibration_m[i * 3 + k][j * 10 + 8] = A[k][3] * q[5] + A[k][5] * q[3];
					_Calibration_m[i * 3 + k][j * 10 + 9] = A[k][4] * q[5] + A[k][5] * q[4];

					_Calibration_m[i * 3 + k][j * 10] += B[k][0] * v[0] + B[k][1] * v[1] + B[k][2] * v[2];
					_Calibration_m[i * 3 + k][j * 10 + 1] += B[k][1] * v[5] + B[k][5] * v[1] - B[k][2] * v[4] - B[k][4] * v[2];
					_Calibration_m[i * 3 + k][j * 10 + 2] += B[k][2] * v[3] + B[k][3] * v[2] - B[k][0] * v[5] - B[k][5] * v[0];
					_Calibration_m[i * 3 + k][j * 10 + 3] += B[k][0] * v[4] + B[k][4] * v[0] - B[k][1] * v[3] - B[k][3] * v[1];
					_Calibration_m[i * 3 + k][j * 10 + 4] += B[k][3] * v[3];
					_Calibration_m[i * 3 + k][j * 10 + 5] += B[k][4] * v[4];
					_Calibration_m[i * 3 + k][j * 10 + 6] += B[k][5] * v[5];
					_Calibration_m[i * 3 + k][j * 10 + 7] += B[k][3] * v[4] + B[k][4] * v[3];
					_Calibration_m[i * 3 + k][j * 10 + 8] += B[k][3] * v[5] + B[k][5] * v[3];
					_Calibration_m[i * 3 + k][j * 10 + 9] += B[k][4] * v[5] + B[k][5] * v[4];
				}

				double test1[10], test2[3];
				test1[0] = pPrt->GetPrtImPtr()[0];
				test1[1] = pPrt->GetPrtImPtr()[11];
				test1[2] = pPrt->GetPrtImPtr()[15];
				test1[3] = pPrt->GetPrtImPtr()[4];
				test1[4] = pPrt->GetPrtImPtr()[21];
				test1[5] = pPrt->GetPrtImPtr()[28];
				test1[6] = pPrt->GetPrtImPtr()[35];
				test1[7] = pPrt->GetPrtImPtr()[27];
				test1[8] = pPrt->GetPrtImPtr()[33];
				test1[9] = pPrt->GetPrtImPtr()[34];

				s_dgemm(3, 1, 10, 1, &_Calibration_m[i * 3][j*10], 60, test1, 1, 0, test2, 1);

				dsp(test2, 3, 1);
				
				//if (pPrt == pP3b)
				//{

				//	dsp(test1, 10, 1);
				//	dsp(test2, 3, 1);
				//	dsp(fplen + i * 3, 3, 1);
				//}
				///*static double test3[6];
				//s_daxpy(6, -1, pPrt->GetPrtFgPtr(), 1, test3, 1);
				//s_daxpy(6, 1, pPrt->GetPrtFvPtr(), 1, test3, 1);
				//s_dgemm(6, 1, 6, 1, pPrt->GetPrtImPtr(), 6, pPrt->GetPrtAccPtr(), 1, 1, test3, 1);
				//s_dgemm(3, 1, 6, 1, &_inv_of_C[33][j * 6], 36, test3, 1, 0, test1, 1);
				//dsp(test1, 3, 1);

				//dsp(&this->_c_M[30][0], 6, 4);
				//dsp(test3, 6, 1);
			}
		}
		
		vector<double[3]> _tem_Calibration_m(n);

		vector<double[3]> s(n);
		vector<int[3]> rank(n);
		double rcond;

		memcpy(_tem_Calibration_m.data(), _Calibration_m.data(), sizeof(double)*n * 3 * 60);

		s_dgelsd(n * 3, 60, 1, &_tem_Calibration_m[0][0], 60, &_Calibration_fce[0][0], 1, &s[0][0], rcond, &rank[0][0]);


		cout << "p1a:" << endl;
		dsp(_Calibration_fce[0], 10, 1);
		cout << "p2a:" << endl;
		dsp(_Calibration_fce[1], 10, 1);
		cout << "p3a:" << endl;
		dsp(_Calibration_fce[2], 10, 1);
		cout << "Thigh:" << endl;
		dsp(_Calibration_fce[3], 10, 1);
		cout << "p2b:" << endl;
		dsp(_Calibration_fce[4], 10, 1);
		cout << "p3b:" << endl;
		dsp(_Calibration_fce[5], 10, 1);
		*/
		//static double real_mass[60];
		//static double test_fce[210];

		//for (int j = 0; j < 6; j++)
		//{
		//	PART *pPrt=0;

		//	switch (j)
		//	{
		//	case 0:
		//		pPrt = pP1a;
		//		break;
		//	case 1:
		//		pPrt = pP2a;
		//		break;
		//	case 2:
		//		pPrt = pP3a;
		//		break;
		//	case 3:
		//		pPrt = pThigh;
		//		break;
		//	case 4:
		//		pPrt = pP2b;
		//		break;
		//	case 5:
		//		pPrt = pP3b;
		//		break;
		//	}

		//	real_mass[j * 10] = pPrt->GetPrtImPtr()[0];
		//	real_mass[j * 10 + 1] = pPrt->GetPrtImPtr()[11];
		//	real_mass[j * 10 + 2] = pPrt->GetPrtImPtr()[15];
		//	real_mass[j * 10 + 3] = pPrt->GetPrtImPtr()[4];
		//	real_mass[j * 10 + 4] = pPrt->GetPrtImPtr()[21];
		//	real_mass[j * 10 + 5] = pPrt->GetPrtImPtr()[28];
		//	real_mass[j * 10 + 6] = pPrt->GetPrtImPtr()[35];
		//	real_mass[j * 10 + 7] = pPrt->GetPrtImPtr()[27];
		//	real_mass[j * 10 + 8] = pPrt->GetPrtImPtr()[33];
		//	real_mass[j * 10 + 9] = pPrt->GetPrtImPtr()[34];
		//}

		////dsp(real_mass, 60, 1);

		//s_dgemm(n * 3, 1, 60, 1, *_Calibration_m, 60, _Calibration_fce, 1, 0, test_fce, 1);

		//dsp(test_fce, 30, 3);
		//dsp(fplen, 30, 3);
	}
	void LEG::_CalCdByPos()
	{
		l1=sqrt(x*x+y*y+z*z-Sfy*Sfy-Sfz*Sfz)-Sfx;
		b1=asin(y/sqrt((l1+Sfx)*(l1+Sfx)+Sfy*Sfy))-asin(Sfy/sqrt((l1+Sfx)*(l1+Sfx)+Sfy*Sfy));
		a1=atan2(Sfz*x-((l1+Sfx)*cos(b1)-Sfy*sin(b1))*z,((l1+Sfx)*cos(b1)-Sfy*sin(b1))*x+Sfz*z);
	}
	void LEG::_CalCdByPlen()
	{
		static std::complex<double>  M,N;
		static std::complex<double>  K1, K2, K3;
		static std::complex<double>  p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
		static std::complex<double>  A, B, C;
		
		static double  X;

		M=(2*(l1*l1+H1*H1+D1*D1+H2*H2+D2*D2)-l2*l2-l3*l3)/4;
		N=(l2*l2-l3*l3)/4;

		K1=-2.0*M/(H1*sqrt(l1*l1+H2*H2));
		K2=(M*M-D1*D1*D2*D2)/(H1*H1*(l1*l1+H2*H2))-1.0;
		K3=-(M*M-D1*D1*D2*D2)/(H1*H1*(l1*l1+H2*H2))-D2*D2*N*N/(H1*H1*(l1*l1+H2*H2)*(l1*l1+H2*H2));

		//方程为：
		//x^4+K1*x^3+K2*x^2-K1*x+K3=0
		//solve 4th equation
		p1=K1+(K1*K2)/2.0-K1*K1*K1/8.0;
		p2=p1*p1;
		p3=K2-3.0*K1*K1/8.0;
		p4=K3+K1*K1*K2/16.0+K1*K1/4.0-(3.0*K1*K1*K1*K1)/256.0;
		p5=sqrt(4.0*p2*p3*p3*p3-16.0*p3*p3*p3*p3*p4+27.0*p2*p2+128.0*p3*p3*p4*p4-256.0*p4*p4*p4-144.0*p2*p3*p4);
		p6=p3*p3*p3/27.0+sqrt(3.0)*p5/18.0+p2/2.0-(4.0*p3*p4)/3.0;
		p7=pow(p6,1.0/3.0);
		p8=sqrt(9.0*p7*p7+p3*p3-6.0*p3*p7+12.0*p4);
		p9=54.0*p1*sqrt(p6);
		p10=- p3*p3*p8 - 12.0*p4*p8 - 12.0*p3*p7*p8 - 9.0*p7*p7*p8;

		A=p8/(6.0*sqrt(p7));
		B=sqrt(p9+p10)/(6.0*sqrt(p7*p8));
		C=sqrt(-p9+p10)/(6.0*sqrt(p7*p8));

		//方程的四个根为：
		//root1 = -K1/4.0 + A + B;
		//root2 = -K1/4.0 + A - B;
		//root3 = -K1/4.0 - A + C;
		//root4 = -K1/4.0 - A - C;
	
		X=(-K1/4.0 - A + C).real();
		//solve finished

		b1=asin(X)-atan(H2/l1);
		a1=asin(N.real()/(D1*(l1*cos(b1)-H2*sin(b1))));
	}
	void LEG::_CalCdByPlen2()
	{
		static double T1, T2, F1, F2, dq1, dq2, Ja, Jb, Jc, Jd;
		static int i;

		a1=0;b1=0;

		T1 = (U2x*U2x + U2y*U2y + U2z*U2z + (l1 + S2x)*(l1 + S2x) + S2y*S2y + S2z*S2z - l2*l2)/2;
		T2 = (U3x*U3x + U3y*U3y + U3z*U3z + (l1 + S3x)*(l1 + S3x) + S3y*S3y + S3z*S3z - l3*l3)/2;

		for (i = 0; i < 10; ++i)
		{
			sa1 = sin(a1);
			ca1 = cos(a1);
			sb1 = sin(b1);
			cb1 = cos(b1);
			
			Ja = -U2x*(S2x + l1)*sa1*cb1 + U2x*S2y*sa1*sb1 + U2x*S2z*ca1 - U2z*(S2x + l1)*ca1*cb1 + U2z*S2y*ca1*sb1 - U2z*S2z*sa1;
			Jb = -U2x*(S2x + l1)*ca1*sb1 - U2x*S2y*ca1*cb1 + U2y*(S2x + l1)*cb1 - U2y*S2y*sb1 + U2z*(S2x + l1)*sa1*sb1 + U2z*S2y*sa1*cb1;
			Jc = -U3x*(S3x + l1)*sa1*cb1 + U3x*S3y*sa1*sb1 + U3x*S3z*ca1 - U3z*(S3x + l1)*ca1*cb1 + U3z*S3y*ca1*sb1 - U3z*S3z*sa1;
			Jd = -U3x*(S3x + l1)*ca1*sb1 - U3x*S3y*ca1*cb1 + U3y*(S3x + l1)*cb1 - U3y*S3y*sb1 + U3z*(S3x + l1)*sa1*sb1 + U3z*S3y*sa1*cb1;

			F1 = U2x*(S2x + l1)*ca1*cb1 - U2x*S2y*ca1*sb1 + U2x*S2z*sa1 
				+ U2y*(S2x + l1)*sb1 + U2y*S2y*cb1 
				- U2z*(S2x + l1)*sa1*cb1 + U2z*S2y*sa1*sb1 + U2z*S2z*ca1;
			F2 = U3x*(S3x + l1)*ca1*cb1 - U3x*S3y*ca1*sb1 + U3x*S3z*sa1
				+ U3y*(S3x + l1)*sb1 + U3y*S3y*cb1
				- U3z*(S3x + l1)*sa1*cb1 + U3z*S3y*sa1*sb1 + U3z*S3z*ca1;

			dq1 = (Jd*(T1 - F1) - Jb*(T2 - F2)) / (Ja*Jd - Jb*Jc);
			dq2 = (-Jc*(T1 - F1) + Ja*(T2 - F2)) / (Ja*Jd - Jb*Jc);

			a1 = a1 + dq1;
			b1 = b1 + dq2;
		}

		this->l1 = l1;

	}
	void LEG::_CalVarByCd()
	{
		sa1 = sin(a1);
		ca1 = cos(a1);
		sb1 = sin(b1);
		cb1 = cos(b1);
				
		x= (l1+Sfx)*ca1*cb1 - Sfy*ca1*sb1 + Sfz*sa1;
		y= (l1+Sfx)*sb1    + Sfy*cb1    + 0;
		z=-(l1+Sfx)*sa1*cb1 + Sfy*sa1*sb1 + Sfz*ca1;
	
		x2= (l1+S2x)*ca1*cb1 - S2y*ca1*sb1 + S2z*sa1 - U2x;
		y2= (l1+S2x)*sb1    + S2y*cb1    + 0      - U2y;
		z2=-(l1+S2x)*sa1*cb1 + S2y*sa1*sb1 + S2z*ca1 - U2z;

		x3= (l1+S3x)*ca1*cb1 - S3y*ca1*sb1 + S3z*sa1 - U3x;
		y3= (l1+S3x)*sb1    + S3y*cb1    + 0      - U3y;
		z3=-(l1+S3x)*sa1*cb1 + S3y*sa1*sb1 + S3z*ca1 - U3z;

		l2=sqrt(x2*x2+y2*y2+z2*z2);
		b2=asin(y2/l2);
		a2=-atan(z2/x2);

		l3=sqrt(x3*x3+y3*y3+z3*z3);
		b3=asin(y3/l3);
		a3=-atan(z3/x3);

		sa2 = sin(a2);
		ca2 = cos(a2);
		sb2 = sin(b2);
		cb2 = cos(b2);
		sa3 = sin(a3);
		ca3 = cos(a3);
		sb3 = sin(b3);
		cb3 = cos(b3);

		J1[0][0] = z;
		J1[0][1] = -y*ca1;
		J1[0][2] = ca1*cb1;
		J1[1][0] = 0;
		J1[1][1] = x*ca1-z*sa1;
		J1[1][2] = sb1;
		J1[2][0] = -x;
		J1[2][1] = y*sa1;
		J1[2][2] = -sa1*cb1;

		k21 = -U2x*(S2x + l1)*sa1*cb1 + U2x*S2y*sa1*sb1 + U2x*S2z*ca1 - U2z*(S2x + l1)*ca1*cb1 + U2z*S2y*ca1*sb1 - U2z*S2z*sa1;
		k22 = -U2x*(S2x + l1)*ca1*sb1 - U2x*S2y*ca1*cb1 + U2y*(S2x + l1)*cb1 - U2y*S2y*sb1 + U2z*(S2x + l1)*sa1*sb1 + U2z*S2y*sa1*cb1;
		k23 = U2x*ca1*cb1 + U2y*sb1 - U2z*sa1*cb1 - (l1 + S2x);

		k31 = -U3x*(S3x + l1)*sa1*cb1 + U3x*S3y*sa1*sb1 + U3x*S3z*ca1 - U3z*(S3x + l1)*ca1*cb1 + U3z*S3y*ca1*sb1 - U3z*S3z*sa1;
		k32 = -U3x*(S3x + l1)*ca1*sb1 - U3x*S3y*ca1*cb1 + U3y*(S3x + l1)*cb1 - U3y*S3y*sb1 + U3z*(S3x + l1)*sa1*sb1 + U3z*S3y*sa1*cb1;
		k33 = U3x*ca1*cb1 + U3y*sb1 - U3z*sa1*cb1 - (l1 + S3x);

		J2[0][0] = 0;
		J2[0][1] = 0;
		J2[0][2] = 1;
		J2[1][0] = -k21 / l2;
		J2[1][1] = -k22 / l2;
		J2[1][2] = -k23 / l2;
		J2[2][0] = -k31 / l3;
		J2[2][1] = -k32 / l3;
		J2[2][2] = -k33 / l3;
	}
	void LEG::_CalPartByVar()
	{
		static double pm[4][4], pm1[4][4], pm2[4][4];
		
		pBase->Update();

		double ep[6] = { PI / 2, a1, -PI / 2 + b1 ,0,0,0};
		s_ep2pm(ep,*pm);
		s_pm_dot_pm(pBase->GetPmPtr(), *pm, *pm1);
		pP1a->SetPm(*pm1);

		double ep1[6] = { 0, 0, 0, l1, 0, 0 };
		s_ep2pm(ep1, *pm);
		s_pm_dot_pm(pP1a->GetPmPtr(), *pm, *pm1);
		pThigh->SetPm(*pm1);

		double ep2[6] = { PI / 2, a2, -PI / 2 + b2, U2x, U2y, U2z };
		s_ep2pm(ep2, *pm);
		s_pm_dot_pm(pBase->GetPmPtr(), *pm, *pm1);
		pP2a->SetPm(*pm1);

		double ep3[6] = { 0, 0, 0, l2, 0, 0 };
		s_ep2pm(ep3, *pm);
		s_pm_dot_pm(pP2a->GetPmPtr(), *pm, *pm1);
		pP2b->SetPm(*pm1);

		double ep4[6] = { PI / 2, a3, -PI / 2 + b3, U3x, U3y, U3z };
		s_ep2pm(ep4, *pm);
		s_pm_dot_pm(pBase->GetPmPtr(), *pm, *pm1);
		pP3a->SetPm(*pm1);

		double ep5[6] = { 0, 0, 0, l3, 0, 0 };
		s_ep2pm(ep5, *pm);
		s_pm_dot_pm(pP3a->GetPmPtr(), *pm, *pm1);
		pP3b->SetPm(*pm1);

		/*更新驱动位置*/
		pM1->SetP_m(&l1);
		pM2->SetP_m(&l2);
		pM3->SetP_m(&l3);

		/*地面球铰的位置*/
		pSfi->Update();
		memcpy(const_cast<double *>(pSfj->GetPrtPmPtr()), pSfi->GetPmPtr(), 16 * sizeof(double));
	}
	void LEG::_CalVcdByVpos()
	{
		vl1 = (x*vx + y*vy + z*vz) / (l1 + Sfx);
		vb1 = (vy - vl1*sb1) / (x*ca1 - z*sa1);
		va1 = (vx*sa1 + vz*ca1) / (-x*ca1 + z*sa1);
	}
	void LEG::_CalVcdByVplen()
	{
		static double K1,K2;

		K1 =  - l2*vl2 - k23*vl1;
		K2 =  - l3*vl3 - k33*vl1;

		va1 = (k32*K1 - k22*K2) / (k21*k32 - k22*k31);
		vb1 = (-k31*K1 + k21*K2) / (k21*k32 - k22*k31);
		this->vl1=vl1;
	}
	void LEG::_CalVvarByVcd()
	{
		vx= z*va1  - y*ca1*vb1             + ca1*cb1*vl1;
		vy= 0      + (x*ca1-z*sa1)*vb1 + sb1*vl1;
		vz= -x*va1 + y*sa1*vb1             - sa1*cb1*vl1;
	
		vx2= (z2+U2z)*va1  - (y2+U2y)*ca1*vb1                    + ca1*cb1*vl1;
		vy2= 0             + ((x2+U2x)*ca1-(z2+U2z)*sa1)*vb1 + sb1*vl1;
		vz2= -(x2+U2x)*va1 + (y2+U2y)*sa1*vb1                    - sa1*cb1*vl1;

		vl2=(x2*vx2+y2*vy2+z2*vz2)/l2;
		vb2=(vy2-vl2*sb2)/(l2*cb2);
		va2=(vx2*sa2+vz2*ca2)/(-x2*ca2+z2*sa2);	
	
		vx3= (z3+U3z)*va1  - (y3+U3y)*ca1*vb1                    + ca1*cb1*vl1;
		vy3= 0             + ((x3+U3x)*ca1-(z3+U3z)*sa1)*vb1 + sb1*vl1;
		vz3= -(x3+U3x)*va1 + (y3+U3y)*sa1*vb1                    - sa1*cb1*vl1;

		vl3=(x3*vx3+y3*vy3+z3*vz3)/l3;
		vb3=(vy3-vl3*sb3)/(l3*cb3);
		va3=(vx3*sa3+vz3*ca3)/(-x3*ca3+z3*sa3);

		pa1 = ca1*va1;
		qa1 = -sa1*va1;
		pb1 = cb1*vb1;
		qb1 = -sb1*vb1;
		pa2 = ca2*va2;
		qa2 = -sa2*va2;
		pb2 = cb2*vb2;
		qb2 = -sb2*vb2;
		pa3 = ca3*va3;
		qa3 = -sa3*va3;
		pb3 = cb3*vb3;
		qb3 = -sb3*vb3;

		H11 = sa1*pb1 + pa1*sb1;
		H12 = sa1*qb1 + pa1*cb1;
		H21 = ca1*pb1 + qa1*sb1;
		H22 = ca1*qb1 + qa1*cb1;

		vk21 = -U2x*(S2x + l1)*H12 + U2x*S2y*H11 + U2x*S2z*qa1
			- U2z*(S2x + l1)*H22 + U2z*S2y*H21 - U2z*S2z*pa1
			- U2x*sa1*cb1*vl1 - U2z*ca1*cb1*vl1;
		vk22 = -U2x*(S2x + l1)*H21 - U2x*S2y*H22
			+ U2y*(S2x + l1)*qb1 - U2y*S2y*pb1
			+ U2z*(S2x + l1)*H11 + U2z*S2y*H12
			- U2x*ca1*sb1*vl1 + U2y*cb1*vl1 + U2z*sa1*sb1*vl1;
		vk23 = U2x*H22 + U2y*pb1 - U2z*H12 - vl1;
		vk31 = -U3x*(S3x + l1)*H12 + U3x*S3y*H11 + U3x*S3z*qa1
			- U3z*(S3x + l1)*H22 + U3z*S3y*H21 - U3z*S3z*pa1
			- U3x*sa1*cb1*vl1 - U3z*ca1*cb1*vl1;
		vk32 = -U3x*(S3x + l1)*H21 - U3x*S3y*H22
			+ U3y*(S3x + l1)*qb1 - U3y*S3y*pb1
			+ U3z*(S3x + l1)*H11 + U3z*S3y*H12
			- U3x*ca1*sb1*vl1 + U3y*cb1*vl1 + U3z*sa1*sb1*vl1;
		vk33 = U3x*H22 + U3y*pb1 - U3z*H12 - vl1;

		vJ1[0][0] = vz;
		vJ1[0][1] = -vy*ca1-y*qa1;
		vJ1[0][2] = H22;
		vJ1[1][0] = 0;
		vJ1[1][1] = vx*ca1+x*qa1-vz*sa1-z*pa1;
		vJ1[1][2] = pb1;
		vJ1[2][0] = -vx;
		vJ1[2][1] = vy*sa1+y*pa1;
		vJ1[2][2] = -H12;

		vJ2[0][0] = 0;
		vJ2[0][1] = 0;
		vJ2[0][2] = 0;
		vJ2[1][0] = -(vk21*l2 - k21*vl2) / (l2*l2);
		vJ2[1][1] = -(vk22*l2 - k22*vl2) / (l2*l2);
		vJ2[1][2] = -(vk23*l2 - k23*vl2) / (l2*l2);
		vJ2[2][0] = -(vk31*l3 - k31*vl3) / (l3*l3);
		vJ2[2][1] = -(vk32*l3 - k32*vl3) / (l3*l3);
		vJ2[2][2] = -(vk33*l3 - k33*vl3) / (l3*l3);

	}
	void LEG::_CalVpartByVvar()
	{
		static double v_G[6], v_L[6];

		/*p1a*/
		v_L[0] = 0;
		v_L[1] = 0;
		v_L[2] = 0;
		v_L[3] = sa1*vb1;
		v_L[4] = va1;
		v_L[5] = ca1*vb1;

		s_v2v(pBase->GetPmPtr(), pBase->GetVelPtr(), v_L, v_G);
		pP1a->SetVel(v_G);

		/*thigh*/
		v_L[0] += ca1*cb1*vl1;
		v_L[1] += sb1*vl1;
		v_L[2] += -sa1*cb1*vl1;

		s_v2v(pBase->GetPmPtr(), pBase->GetVelPtr(), v_L, v_G);
		pThigh->SetVel(v_G);

		/*p2a*/
		v_L[0] = -U2z*va2 + U2y*ca2*vb2;
		v_L[1] = +U2z*sa2*vb2 - U2x*ca2*vb2;
		v_L[2] = -U2y*sa2*vb2 + U2x*va2;
		v_L[3] = sa2*vb2;
		v_L[4] = va2;
		v_L[5] = ca2*vb2;

		s_v2v(pBase->GetPmPtr(), pBase->GetVelPtr(), v_L, v_G);
		pP2a->SetVel(v_G);

		/*p2b*/
		v_L[0] += ca2*cb2*vl2;
		v_L[1] += sb2*vl2;
		v_L[2] += -sa2*cb2*vl2;

		s_v2v(pBase->GetPmPtr(), pBase->GetVelPtr(), v_L, v_G);
		pP2b->SetVel(v_G);

		/*p3a*/
		v_L[0] = -U3z*va3 + U3y*ca3*vb3;
		v_L[1] = +U3z*sa3*vb3 - U3x*ca3*vb3;
		v_L[2] = -U3y*sa3*vb3 + U3x*va3;
		v_L[3] = sa3*vb3;
		v_L[4] = va3;
		v_L[5] = ca3*vb3;

		s_v2v(pBase->GetPmPtr(), pBase->GetVelPtr(), v_L, v_G);
		pP3a->SetVel(v_G);

		/*p3b*/
		v_L[0] += ca3*cb3*vl3;
		v_L[1] += sb3*vl3;
		v_L[2] += -sa3*cb3*vl3;

		s_v2v(pBase->GetPmPtr(), pBase->GetVelPtr(), v_L, v_G);
		pP3b->SetVel(v_G);


		pM1->SetV_m(&vl1);
		pM2->SetV_m(&vl2);
		pM3->SetV_m(&vl3);
	}
	void LEG::_CalAcdByApos()
	{
		static double xd, yd, zd;

		xd = ax - vJ1[0][0] * va1 - vJ1[0][1] * vb1 - vJ1[0][2] * vl1;
		yd = ay - vJ1[1][0] * va1 - vJ1[1][1] * vb1 - vJ1[1][2] * vl1;
		zd = az - vJ1[2][0] * va1 - vJ1[2][1] * vb1 - vJ1[2][2] * vl1;

		al1 = (x*xd + y*yd + z*zd) / (l1 + Sfx);
		ab1 = (yd - al1*sb1) / (x*ca1 - z*sa1);
		aa1 = (xd*sa1 + zd*ca1) / (-x*ca1 + z*sa1);
	}
	void LEG::_CalAcdByAplen()
	{
		static double vK1, vK2, M1, M2;

		vK1 = -vl2*vl2 - l2*al2 - vk23*vl1 - k23*al1;
		vK2 = -vl3*vl3 - l3*al3 - vk33*vl1 - k33*al1;

		M1 = vK1 - vk21*va1 - vk22*vb1;
		M2 = vK2 - vk31*va1 - vk32*vb1;

		aa1 = (k32*M1 - k22*M2) / (k21*k32 - k22*k31);
		ab1 = (-k31*M1 + k21*M2) / (k21*k32 - k22*k31);
	}
	void LEG::_CalAvarByAcd()
	{
		ax = z*aa1 - y*ca1*ab1 + ca1*cb1*al1
			+ vz*va1 + (-vy*ca1 - y*qa1)*vb1 + H22*vl1;
		ay = (x*ca1 - z*sa1)*ab1 + sb1*al1
			+ (vx*ca1 + x*qa1 - vz*sa1 - z*pa1)*vb1 + pb1*vl1;
		az = -x*aa1 + y*sa1*ab1 - sa1*cb1*al1
			- vx*va1 + vy*sa1*vb1 + y*pa1*vb1 - H12*vl1;

		ax2 = (z2 + U2z)*aa1 - (y2 + U2y)*ca1*ab1 + ca1*cb1*al1
			+ vz2*va1 + (-vy2*ca1 - (y2 + U2y)*qa1)*vb1 + H22*vl1;
		ay2 = ((x2 + U2x)*ca1 - (z2 + U2z)*sa1)*ab1 + sb1*al1
			+ (vx2*ca1 + (x2 + U2x)*qa1 - vz2*sa1 - (z2 + U2z)*pa1)*vb1 + pb1*vl1;
		az2 = (-x2 - U2x)*aa1 + (y2 + U2y)*sa1*ab1 - sa1*cb1*al1
			- vx2*va1 + vy2*sa1*vb1 + (y2 + U2y)*pa1*vb1 -H12*vl1;

		al2 = (x2*ax2 + vx2*vx2 + y2*ay2 + vy2*vy2 + z2*az2 + vz2*vz2 - vl2*vl2) 
			/ l2;
		ab2 = (ay2-pb2*vl2-sb2*al2-vl2*pb2-l2*qb2*vb2)
			/ (l2*cb2);
		aa2 = (pa2*vx2 + sa2*ax2 + ca2*az2 + qa2*vz2 - (-vx2*ca2 - x2*qa2 + vz2*sa2 + z2*pa2)*va2) 
			/ (-x2*ca2 + z2*sa2);

		ax3 = (z3 + U3z)*aa1 - (y3 + U3y)*ca1*ab1 + ca1*cb1*al1
			+ vz3*va1 + (-vy3*ca1 - (y3 + U3y)*qa1)*vb1 + H22*vl1;
		ay3 = ((x3 + U3x)*ca1 - (z3 + U3z)*sa1)*ab1 + sb1*al1
			+ (vx3*ca1 + (x3 + U3x)*qa1 - vz3*sa1 - (z3 + U3z)*pa1)*vb1 + pb1*vl1;
		az3 = (-x3 - U3x)*aa1 + (y3 + U3y)*sa1*ab1 - sa1*cb1*al1
			- vx3*va1 + vy3*sa1*vb1 + (y3 + U3y)*pa1*vb1 - H12*vl1;

		al3 = (x3*ax3 + vx3*vx3 + y3*ay3 + vy3*vy3 + z3*az3 + vz3*vz3 - vl3*vl3)
			/ l3;
		ab3 = (ay3 - pb3*vl3 - sb3*al3 - vl3*pb3 - l3*qb3*vb3)
			/ (l3*cb3);
		aa3 = (pa3*vx3 + sa3*ax3 + ca3*az3 + qa3*vz3 - (-vx3*ca3 - x3*qa3 + vz3*sa3 + z3*pa3)*va3)
			/ (-x3*ca3 + z3*sa3);
	}
	void LEG::_CalApartByAvar()
	{
		static double a_L[6], a_G[6], v_G[6], v_L[6];
		
		/* p1a */
		v_L[0] = 0;
		v_L[1] = 0;
		v_L[2] = 0;
		v_L[3] = sa1*vb1;
		v_L[4] = va1;
		v_L[5] = ca1*vb1;

		a_L[0] = 0;
		a_L[1] = 0;
		a_L[2] = 0;
		a_L[3] = pa1*vb1 + sa1*ab1;
		a_L[4] = aa1;
		a_L[5] = qa1*vb1 + ca1*ab1;

		s_a2a(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), v_L, a_L, a_G, v_G);
		pP1a->SetAcc(a_G);

		/* Thigh */
		v_L[0] += ca1*cb1*vl1;
		v_L[1] += sb1*vl1;
		v_L[2] += -sa1*cb1*vl1;

		a_L[0] = H22*vl1 + ca1*cb1*al1;
		a_L[1] = cb1*vb1*vl1 + sb1*al1;
		a_L[2] = -H12*vl1 - sa1*cb1*al1;

		s_a2a(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), v_L, a_L, a_G, v_G);

		pThigh->SetAcc(a_G);

		/* P2a */
		v_L[0] = -U2z*va2 + U2y*ca2*vb2;
		v_L[1] = +U2z*sa2*vb2 - U2x*ca2*vb2;
		v_L[2] = -U2y*sa2*vb2 + U2x*va2;
		v_L[3] = sa2*vb2;
		v_L[4] = va2;
		v_L[5] = ca2*vb2;

		a_L[0] = -U2z*aa2 + U2y*qa2*vb2 + U2y*ca2*ab2;
		a_L[1] = U2z*pa2*vb2 + U2z*sa2*ab2 - U2x*qa2*vb2 - U2x*ca2*ab2;
		a_L[2] = -U2y*pa2*vb2 - U2y*sa2*ab2 + U2x*aa2;
		a_L[3] = pa2*vb2 + sa2*ab2;
		a_L[4] = aa2;
		a_L[5] = qa2*vb2 + ca2*ab2;

		s_a2a(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), v_L, a_L, a_G, v_G);
		pP2a->SetAcc(a_G);
		/* P2b */
		v_L[0] += ca2*cb2*vl2;
		v_L[1] += sb2*vl2;
		v_L[2] += -sa2*cb2*vl2;

		a_L[0] += qa2*cb2*vl2 + ca2*qb2*vl2 + ca2*cb2*al2;
		a_L[1] += pb2*vl2 + sb2*al2;
		a_L[2] += -pa2*cb2*vl2 - sa2*qb2*vl2 - sa2*cb2*al2;

		s_a2a(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), v_L, a_L, a_G, v_G);
		pP2b->SetAcc(a_G);
		/* P3a */
		v_L[0] = -U3z*va3 + U3y*ca3*vb3;
		v_L[1] = +U3z*sa3*vb3 - U3x*ca3*vb3;
		v_L[2] = -U3y*sa3*vb3 + U3x*va3;
		v_L[3] = sa3*vb3;
		v_L[4] = va3;
		v_L[5] = ca3*vb3;

		a_L[0] = -U3z*aa3 + U3y*qa3*vb3 + U3y*ca3*ab3;
		a_L[1] = U3z*pa3*vb3 + U3z*sa3*ab3 - U3x*qa3*vb3 - U3x*ca3*ab3;
		a_L[2] = -U3y*pa3*vb3 - U3y*sa3*ab3 + U3x*aa3;
		a_L[3] = pa3*vb3 + sa3*ab3;
		a_L[4] = aa3;
		a_L[5] = qa3*vb3 + ca3*ab3;

		s_a2a(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), v_L, a_L, a_G, v_G);
		pP3a->SetAcc(a_G);
		/* P3b */
		v_L[0] += ca3*cb3*vl3;
		v_L[1] += sb3*vl3;
		v_L[2] += -sa3*cb3*vl3;

		a_L[0] += qa3*cb3*vl3 + ca3*qb3*vl3 + ca3*cb3*al3;
		a_L[1] += pb3*vl3 + sb3*al3;
		a_L[2] += -pa3*cb3*vl3 - sa3*qb3*vl3 - sa3*cb3*al3;

		s_a2a(pBase->GetPmPtr(), pBase->GetVelPtr(), pBase->GetAccPtr(), v_L, a_L, a_G, v_G);
		pP3b->SetAcc(a_G);

		pM1->SetA_m(&al1);
		pM2->SetA_m(&al2);
		pM3->SetA_m(&al3);
	}
	
	ROBOT::ROBOT()
		: LF_Leg("LF", this)
		, LM_Leg("LM", this)
		, LR_Leg("LR", this)
		, RF_Leg("RF", this)
		, RM_Leg("RM", this)
		, RR_Leg("RR", this)
		, pLF(&LF_Leg)
		, pLM(&LM_Leg)
		, pLR(&LR_Leg)
		, pRF(&RF_Leg)
		, pRM(&RM_Leg)
		, pRR(&RR_Leg)
	{
	}
	ROBOT::~ROBOT()
	{
	}
	
	int ROBOT::LoadXML(const char *filename)
	{
		MODEL::LoadXML(filename);

		char temName[40];

		/*Update Parts*/
		pBody = GetPart("MainBody");
			
		for (int j = 0; j < 6; ++j)
		{
			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P1a");
			pLegs[j]->pP1a = GetPart(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P2a");
			pLegs[j]->pP2a = GetPart(temName);
			
			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P3a");
			pLegs[j]->pP3a = GetPart(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_Thigh");
			pLegs[j]->pThigh = GetPart(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P2b");
			pLegs[j]->pP2b = GetPart(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P3b");
			pLegs[j]->pP3b = GetPart(temName);
		}

		/*Update Markers*/
		pBodyCenter = pBody->GetMarker("BodyCenter");

		for (int j = 0; j < 6; ++j)
		{
			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_Base");
			pLegs[j]->pBase = pBody->GetMarker(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_U1i");
			pLegs[j]->pU1i = pBody->GetMarker(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_U2i");
			pLegs[j]->pU2i = pBody->GetMarker(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_U3i");
			pLegs[j]->pU3i = pBody->GetMarker(temName);

			pLegs[j]->pU1j = pLegs[j]->pP1a->GetMarker("U1j");
			pLegs[j]->pU2j = pLegs[j]->pP2a->GetMarker("U2j");
			pLegs[j]->pU3j = pLegs[j]->pP3a->GetMarker("U3j");

			pLegs[j]->pP1i = pLegs[j]->pThigh->GetMarker("P1i");
			pLegs[j]->pP2i = pLegs[j]->pP2b->GetMarker("P2i");
			pLegs[j]->pP3i = pLegs[j]->pP3b->GetMarker("P3i");

			pLegs[j]->pP1j = pLegs[j]->pP1a->GetMarker("P1j");
			pLegs[j]->pP2j = pLegs[j]->pP2a->GetMarker("P2j");
			pLegs[j]->pP3j = pLegs[j]->pP3a->GetMarker("P3j");

			pLegs[j]->pSfi = pLegs[j]->pThigh->GetMarker("Sfi");
			pLegs[j]->pS2i = pLegs[j]->pThigh->GetMarker("S2i");
			pLegs[j]->pS3i = pLegs[j]->pThigh->GetMarker("S3i");

			pLegs[j]->pS2j = pLegs[j]->pP2b->GetMarker("S2j");
			pLegs[j]->pS3j = pLegs[j]->pP3b->GetMarker("S3j");

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_Sfj");
			pLegs[j]->pSfj = pGround->GetMarker(temName);
		}

		/*Update Joints*/
		for (int j = 0; j < 6; ++j)
		{
			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_U1");
			pLegs[j]->pU1 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_U2");
			pLegs[j]->pU2 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_U3");
			pLegs[j]->pU3 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P1");
			pLegs[j]->pP1 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P2");
			pLegs[j]->pP2 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_P3");
			pLegs[j]->pP3 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_Sf");
			pLegs[j]->pSf = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_S2");
			pLegs[j]->pS2 = GetJoint(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_S3");
			pLegs[j]->pS3 = GetJoint(temName);
		}

		/*Update Motions*/
		for (int j = 0; j < 6; ++j)
		{
			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_M1");
			pLegs[j]->pM1 = GetMotion(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_M2");
			pLegs[j]->pM2 = GetMotion(temName);

			strcpy(temName, pLegs[j]->_Name.data());
			strcat(temName, "_M3");
			pLegs[j]->pM3 = GetMotion(temName);
		}

		/*Update Dimension Variables*/
		for (int i = 0; i < 6; ++i)
		{
			double pm[4][4];
			s_inv_pm_dot_pm(pLegs[i]->pBase->GetPrtPmPtr(), pLegs[i]->pU2i->GetPrtPmPtr(), *pm);
			*const_cast<double *>(&pLegs[i]->U2x) = pm[0][3];
			*const_cast<double *>(&pLegs[i]->U2y) = pm[1][3];
			*const_cast<double *>(&pLegs[i]->U2z) = pm[2][3];
			s_inv_pm_dot_pm(pLegs[i]->pBase->GetPrtPmPtr(), pLegs[i]->pU3i->GetPrtPmPtr(), *pm);
			*const_cast<double *>(&pLegs[i]->U3x) = pm[0][3];
			*const_cast<double *>(&pLegs[i]->U3y) = pm[1][3];
			*const_cast<double *>(&pLegs[i]->U3z) = pm[2][3];
			*const_cast<double *>(&pLegs[i]->S2x) = pLegs[i]->pS2i->GetPrtPmPtr()[3];
			*const_cast<double *>(&pLegs[i]->S2y) = pLegs[i]->pS2i->GetPrtPmPtr()[7];
			*const_cast<double *>(&pLegs[i]->S2z) = pLegs[i]->pS2i->GetPrtPmPtr()[11];
			*const_cast<double *>(&pLegs[i]->S3x) = pLegs[i]->pS3i->GetPrtPmPtr()[3];
			*const_cast<double *>(&pLegs[i]->S3y) = pLegs[i]->pS3i->GetPrtPmPtr()[7];
			*const_cast<double *>(&pLegs[i]->S3z) = pLegs[i]->pS3i->GetPrtPmPtr()[11];
			*const_cast<double *>(&pLegs[i]->Sfx) = pLegs[i]->pSfi->GetPrtPmPtr()[3];
			*const_cast<double *>(&pLegs[i]->Sfy) = pLegs[i]->pSfi->GetPrtPmPtr()[7];
			*const_cast<double *>(&pLegs[i]->Sfz) = pLegs[i]->pSfi->GetPrtPmPtr()[11];

			*const_cast<double *>(&pLegs[i]->D1) = pLegs[i]->U2z;
			*const_cast<double *>(&pLegs[i]->H1) = pLegs[i]->U2y;
			*const_cast<double *>(&pLegs[i]->D2) = pLegs[i]->pS2i->GetPrtPmPtr()[11];
			*const_cast<double *>(&pLegs[i]->H2) = pLegs[i]->pS2i->GetPrtPmPtr()[7];
		}

		return 0;
	}

	void ROBOT::GetPee(double *pEE, const char *RelativeCoodinate) const
	{
		pLF->GetPee(pEE, RelativeCoodinate);
		pLM->GetPee(pEE + 3, RelativeCoodinate);
		pLR->GetPee(pEE + 6, RelativeCoodinate);
		pRF->GetPee(pEE + 9, RelativeCoodinate);
		pRM->GetPee(pEE + 12, RelativeCoodinate);
		pRR->GetPee(pEE + 15, RelativeCoodinate);
	}
	void ROBOT::GetPin(double *pIn) const
	{
		pLF->GetPin(pIn);
		pLM->GetPin(pIn + 3);
		pLR->GetPin(pIn + 6);
		pRF->GetPin(pIn + 9);
		pRM->GetPin(pIn + 12);
		pRR->GetPin(pIn + 15);
	}
	void ROBOT::GetBodyPm(double *bodypm) const
	{
		memcpy(bodypm, pBody->GetPmPtr(), sizeof(double)* 16);
	}
	void ROBOT::GetVee(double *vEE, const char *RelativeCoodinate) const
	{
		pLF->GetVee(vEE, RelativeCoodinate);
		pLM->GetVee(vEE + 3, RelativeCoodinate);
		pLR->GetVee(vEE + 6, RelativeCoodinate);
		pRF->GetVee(vEE + 9, RelativeCoodinate);
		pRM->GetVee(vEE + 12, RelativeCoodinate);
		pRR->GetVee(vEE + 15, RelativeCoodinate);
	}
	void ROBOT::GetVin(double *vIn) const
	{
		pLF->GetVin(vIn);
		pLM->GetVin(vIn + 3);
		pLR->GetVin(vIn + 6);
		pRF->GetVin(vIn + 9);
		pRM->GetVin(vIn + 12);
		pRR->GetVin(vIn + 15);
	}
	void ROBOT::GetBodyVel(double *bodyvel) const
	{
		memcpy(bodyvel, this->pBody->GetVelPtr(), 6 * sizeof(double));
	}
	void ROBOT::GetAee(double *aEE, const char *RelativeCoodinate) const
	{
		pLF->GetAee(aEE, RelativeCoodinate);
		pLM->GetAee(aEE + 3, RelativeCoodinate);
		pLR->GetAee(aEE + 6, RelativeCoodinate);
		pRF->GetAee(aEE + 9, RelativeCoodinate);
		pRM->GetAee(aEE + 12, RelativeCoodinate);
		pRR->GetAee(aEE + 15, RelativeCoodinate);
	}
	void ROBOT::GetAin(double *aIn) const
	{
		pLF->GetAin(aIn);
		pLM->GetAin(aIn + 3);
		pLR->GetAin(aIn + 6);
		pRF->GetAin(aIn + 9);
		pRM->GetAin(aIn + 12);
		pRR->GetAin(aIn + 15);
	}
	void ROBOT::GetBodyAcc(double *bodyacc) const
	{
		memcpy(bodyacc, this->pBody->GetAccPtr(), 6 * sizeof(double));
	}
	void ROBOT::SetPee(const double *pEE, const double *bodyep, const char *RelativeCoodinate)
	{
		if (bodyep != 0)
		{
			s_ep2pm(bodyep, pBody->GetPmPtr());
		}

		if (pEE != 0)
		{
			this->pLF->SetPee(pEE, RelativeCoodinate);
			this->pLM->SetPee(pEE + 3, RelativeCoodinate);
			this->pLR->SetPee(pEE + 6, RelativeCoodinate);
			this->pRF->SetPee(pEE + 9, RelativeCoodinate);
			this->pRM->SetPee(pEE + 12, RelativeCoodinate);
			this->pRR->SetPee(pEE + 15, RelativeCoodinate);
		}

	}
	void ROBOT::SetPin(const double *pIn, const double *bodyep)
	{
		if (bodyep != 0)
		{
			s_ep2pm(bodyep, pBody->GetPmPtr());
		}

		if (pIn != 0)
		{
			this->pLF->SetPin(pIn);
			this->pLM->SetPin(pIn + 3);
			this->pLR->SetPin(pIn + 6);
			this->pRF->SetPin(pIn + 9);
			this->pRM->SetPin(pIn + 12);
			this->pRR->SetPin(pIn + 15);
		}
	}
	void ROBOT::SetVee(const double *vEE, const double *bodyvel, const char *RelativeCoodinate)
	{
		if (bodyvel != 0)
		{
			this->pBody->SetVel(bodyvel);
		}

		if (vEE != 0)
		{
			this->pLF->SetVee(vEE, RelativeCoodinate);
			this->pLM->SetVee(vEE + 3, RelativeCoodinate);
			this->pLR->SetVee(vEE + 6, RelativeCoodinate);
			this->pRF->SetVee(vEE + 9, RelativeCoodinate);
			this->pRM->SetVee(vEE + 12, RelativeCoodinate);
			this->pRR->SetVee(vEE + 15, RelativeCoodinate);
		}
	}
	void ROBOT::SetVin(const double *vIn, const double *bodyvel)
	{
		if (bodyvel != 0)
		{
			this->pBody->SetVel(bodyvel);
		}
		if (vIn != 0)
		{
			this->pLF->SetVin(vIn);
			this->pLM->SetVin(vIn + 3);
			this->pLR->SetVin(vIn + 6);
			this->pRF->SetVin(vIn + 9);
			this->pRM->SetVin(vIn + 12);
			this->pRR->SetVin(vIn + 15);
		}
	}
	void ROBOT::SetAee(const double *aEE, const double *bodyacc, const char *RelativeCoodinate)
	{
		if (bodyacc != 0)
		{
			this->pBody->SetAcc(bodyacc);
		}
		if (aEE != 0)
		{
			this->pLF->SetAee(aEE, RelativeCoodinate);
			this->pLM->SetAee(aEE + 3, RelativeCoodinate);
			this->pLR->SetAee(aEE + 6, RelativeCoodinate);
			this->pRF->SetAee(aEE + 9, RelativeCoodinate);
			this->pRM->SetAee(aEE + 12, RelativeCoodinate);
			this->pRR->SetAee(aEE + 15, RelativeCoodinate);
		}
	}
	void ROBOT::SetAin(const double *aplens, const double *bodyacc)
	{
		if (bodyacc != 0)
		{
			this->pBody->SetAcc(bodyacc);
		}
		if (aplens != 0)
		{
			this->pLF->SetAin(aplens);
			this->pLM->SetAin(aplens + 3);
			this->pLR->SetAin(aplens + 6);
			this->pRF->SetAin(aplens + 9);
			this->pRM->SetAin(aplens + 12);
			this->pRR->SetAin(aplens + 15);
		}
	}

	void ROBOT::GetVelJacInv(double *jac, const char *ActiveMotion) const
	{
		static double J[18][6];

		static double legJac[3][3],cm[3][3];

		static double ee[3];
		static int rowNum,row;

		memset(*J, 0, sizeof(double)* 18 * 6);

		for (int i = 0; i < 6; ++i)
		{
			pLegs[i]->GetPee(ee,"G");
			s_cm3(ee, *cm);
			pLegs[i]->GetVelJacInv(*legJac,0,"L");
			s_dgemmNT(3, 3, 3, -1, *legJac, 3, pLegs[i]->pBase->GetPmPtr(), 4, 0, &J[i * 3][0], 6);
			s_dgemm(3, 3, 3, -1, &J[i * 3][0], 6, *cm, 3, 0, &J[i * 3][3], 6);
		}

		if (ActiveMotion == 0)
		{
			memcpy(jac, *J, sizeof(double)* 18 * 6);
		}
		else
		{
			for (int i = 0; i < 36; i++)//26个字母加10个数字最多可以表达36个电机
			{
				if (ActiveMotion[i] == '\0')
				{
					rowNum = i;
					break;
				}
				

				if ((ActiveMotion[i] <= '9') && (ActiveMotion[i] >= '0'))
				{
					row = ActiveMotion[i] - '0';
				}
				else if ((ActiveMotion[i] <= 'h') && (ActiveMotion[i] >= 'a'))
				{
					row = ActiveMotion[i] - 'a'+10;
				}
				memcpy(&jac[i * 6], &J[row][0], 6 * sizeof(double));
			}
		}
	}
	void ROBOT::GetVelJacDir(double *jac, const char  *ActiveMotion) const
	{
		static double InvJac[18][6],Eye[18][18],s[18],rcond;
		static int rank;

		memset(*Eye, 0, sizeof(double)* 18 * 18);

		rcond = 0.000000000001;

		GetVelJacInv(*InvJac, ActiveMotion);

		int n=std::strlen(ActiveMotion);

		for (int i = 0; i < n; ++i)
		{
			Eye[i][i] = 1;
		}
		/*求逆Jacobian矩阵的广义逆*/
		s_dgelsd(n, 6, n, *InvJac, 6, *Eye, 18, s, rcond, &rank);

		for (int i = 0; i < 6; ++i)
		{
			memcpy(&jac[i*n], &Eye[i][0], n*sizeof(double));
		}
	}

	void ROBOT::SetFixedFeet(const char *fixedLeg, const char *ActiveMotion)
	{
		MOTION** const mots[18] =
		{ &(pLF->pM1), &(pLF->pM2), &(pLF->pM3),
			&(pLM->pM1), &(pLM->pM2), &(pLM->pM3), 
			&(pLR->pM1), &(pLR->pM2), &(pLR->pM3), 
			&(pRF->pM1), &(pRF->pM2), &(pRF->pM3), 
			&(pRM->pM1), &(pRM->pM2), &(pRM->pM3), 
			&(pRR->pM1), &(pRR->pM2), &(pRR->pM3), };
		
		int leg,Mot;

		if (fixedLeg == 0)
		{
			for (int i = 0; i < 6; ++i)
			{
				pLegs[i]->pSf->Activate();
			}
			
			if (ActiveMotion == 0)
			{
				for (auto &i:_motions)
				{
					i->SetMode(MOTION::POS_CONTROL);
				}
			}
			else
			{
				for (auto &i : _motions)
				{
					i->SetMode(MOTION::FCE_CONTROL);
				}
				
				int j = 0;
				while (ActiveMotion[j] != '\0')
				{
					if ((ActiveMotion[j] <= '9') && (ActiveMotion[j] >= '0'))
					{
						Mot = ActiveMotion[j] - '0';
					}
					else if ((ActiveMotion[j] <= 'h') && (ActiveMotion[j] >= 'a'))
					{
						Mot = ActiveMotion[j] - 'a' + 10;
					}
					else
					{
						continue;
					}
					(*mots[Mot])->SetMode(MOTION::POS_CONTROL);
					j++;
				}
			}
		}
		else
		{
			for (int i = 0; i < 6; ++i)
			{
				pLegs[i]->pSf->Deactivate();
			}

			int i = 0;
			while (fixedLeg[i] != '\0')
			{
				if ((fixedLeg[i] <= '5') && (fixedLeg[i] >= '0'))
				{
					leg = fixedLeg[i] - '0';
				}
				else
				{
					continue;
				}
				pLegs[leg]->pSf->Activate();
				i++;
			}
			
			if (ActiveMotion == 0)
			{
				for (auto &i : _motions)
				{
					i->SetMode(MOTION::POS_CONTROL);
				}
			}
			else
			{
				for (auto &i : _motions)
				{
					i->SetMode(MOTION::FCE_CONTROL);
				}

				for (int i = 0; i < 6; ++i)
				{
					if (!pLegs[i]->pSf->GetActive())
					{
						pLegs[i]->pM1->SetMode(MOTION::POS_CONTROL);
						pLegs[i]->pM2->SetMode(MOTION::POS_CONTROL);
						pLegs[i]->pM3->SetMode(MOTION::POS_CONTROL);
					}
				}
				int j = 0;
				while (ActiveMotion[j] != '\0')
				{
					if ((ActiveMotion[j] <= '9') && (ActiveMotion[j] >= '0'))
					{
						Mot = ActiveMotion[j] - '0';
					}
					else if ((ActiveMotion[j] <= 'h') && (ActiveMotion[j] >= 'a'))
					{
						Mot = ActiveMotion[j] - 'a' + 10;
					}
					else
					{
						continue;
					}
					(*mots[Mot])->SetMode(MOTION::POS_CONTROL);
					j++;
				}
			}
		}
	}
	void ROBOT::SetVinWithFixedFeet(const double *vIn)
	{
		static int leg, Mot, dim;
		static double J[18][6];
		static double Jp[18][6], vp[18];//部分雅克比矩阵、部分雅克比矩阵导数、输入速度、输入加速度
		static double legJac[3][3], cm[3][3], R[3][3];
		static double ee[3], vee[3] = {0,0,0};

		static int ipiv[3];

		memset(*J, 0, sizeof(double)* 18 * 6);

		/*计算全部雅克比矩阵及其导数*/
		for (int i = 0; i < 6; ++i)
		{
			//Sf cross
			pLegs[i]->GetPee(ee, "G");
			s_cm3(ee, *cm);
			
			//legjac
			pLegs[i]->GetVelJacInv(*legJac,0,"L");

			//R G2L
			R[0][0] = pLegs[i]->pBase->GetPmPtr()[0];
			R[0][1] = pLegs[i]->pBase->GetPmPtr()[4];
			R[0][2] = pLegs[i]->pBase->GetPmPtr()[8];
			R[1][0] = pLegs[i]->pBase->GetPmPtr()[1];
			R[1][1] = pLegs[i]->pBase->GetPmPtr()[5];
			R[1][2] = pLegs[i]->pBase->GetPmPtr()[9];
			R[2][0] = pLegs[i]->pBase->GetPmPtr()[2];
			R[2][1] = pLegs[i]->pBase->GetPmPtr()[6];
			R[2][2] = pLegs[i]->pBase->GetPmPtr()[10];

			//calculate J
			s_dgemmNT(3, 3, 3, -1, *legJac, 3, pLegs[i]->pBase->GetPmPtr(), 4, 0, &J[i * 3][0], 6);
			s_dgemm(3, 3, 3, -1, &J[i * 3][0], 6, *cm, 3, 0, &J[i * 3][3], 6);
		}

		/*计算支撑腿中位控电机部分的雅克比矩阵及其导数、输入等*/
		dim = 0;
		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->pSf->GetActive())
			{
				if (pLegs[i]->pM1->GetMode() == MOTION::POS_CONTROL)
				{
					memcpy(&Jp[dim][0], &J[i * 3][0], 6 * sizeof(double));
					vp[dim] = pLegs[i]->vl1;
					dim++;
				};
				if (pLegs[i]->pM2->GetMode() == MOTION::POS_CONTROL)
				{
					memcpy(&Jp[dim][0], &J[i * 3 + 1][0], 6 * sizeof(double));
					vp[dim] = pLegs[i]->vl2;
					dim++;
				};
				if (pLegs[i]->pM3->GetMode() == MOTION::POS_CONTROL)
				{
					memcpy(&Jp[dim][0], &J[i * 3 + 2][0], 6 * sizeof(double));
					vp[dim] = pLegs[i]->vl3;
					dim++;
				};
			}
		}

		/*使用部分雅克比矩阵计算身体加速度*/
		static double body_vel[6];
		static double s[18], rcond = 0.0000001;
		static int rank;

		memcpy(body_vel, vp, 6 * sizeof(double));

		s_dgelsd(dim, 6, 1, *Jp, 6, body_vel, 1, s, rcond, &rank);

		pBody->SetVel(body_vel);

		/*设置身体以及各腿加速度*/
		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->pSf->GetActive())
			{
				pLegs[i]->SetVee(vee,"G");
			}
			else
			{
				pLegs[i]->SetVin(&vIn[i * 3]);
			}
		}
	}
	void ROBOT::SetAinWithFixedFeet(const double *aIn)
	{
		static int leg, Mot, dim;
		static double J[18][6], vJ[18][6];
		static double Jp[18][6], vJp[18][6], vp[18], ap[18];//部分雅克比矩阵、部分雅克比矩阵导数、输入速度、输入加速度
		static double legJac[3][3],cm[3][3],vLegJac[3][3],inv_J1[3][3],R[3][3],vR[3][3];
		static double tem[3][3], tem2[3][3], vcm[3][3];
		static double ee[3], vee[3], aee[3] = {0,0,0};

		static int ipiv[3];

		memset(*J, 0, sizeof(double)* 18 * 6);
		memset(*vJ, 0, sizeof(double)* 18 * 6);

		/*计算全部雅克比矩阵及其导数*/
		for (int i = 0; i < 6; ++i)
		{
			//Sf cross
			pLegs[i]->GetPee(ee, "G");
			s_cm3(ee, *cm);
			//legjac
			pLegs[i]->GetVelJacInv(*legJac,0,"L");
			//leg jac dot
			memcpy(*inv_J1, *pLegs[i]->J1,9*sizeof(double));
			s_dgeinv(3, *inv_J1, 3, ipiv);

			s_dgemm(3, 3, 3, 1, *pLegs[i]->vJ1, 3, *inv_J1, 3, 0, *tem, 3);
			s_dgemm(3, 3, 3, 1, *inv_J1, 3, *tem, 3, 0, *tem2, 3);
			s_dgemm(3, 3, 3, -1, *pLegs[i]->J2, 3, *tem2, 3, 0, *vLegJac, 3);
			s_dgemm(3, 3, 3, 1, *pLegs[i]->vJ2, 3, *inv_J1, 3, 1, *vLegJac, 3);
			//R G2L
			R[0][0] = pLegs[i]->pBase->GetPmPtr()[0];
			R[0][1] = pLegs[i]->pBase->GetPmPtr()[4];
			R[0][2] = pLegs[i]->pBase->GetPmPtr()[8];
			R[1][0] = pLegs[i]->pBase->GetPmPtr()[1];
			R[1][1] = pLegs[i]->pBase->GetPmPtr()[5];
			R[1][2] = pLegs[i]->pBase->GetPmPtr()[9];
			R[2][0] = pLegs[i]->pBase->GetPmPtr()[2];
			R[2][1] = pLegs[i]->pBase->GetPmPtr()[6];
			R[2][2] = pLegs[i]->pBase->GetPmPtr()[10];
			//R G2L dot
			memcpy(vee,&pBody->GetVelPtr()[3],sizeof(double)*3);
			s_cm3(vee, *vcm);
			s_dgemmNT(3, 3, 3, 1, *R, 3, *vcm, 3, 0, *vR, 3);

			//calculate J
			s_dgemmNT(3, 3, 3, -1, *legJac, 3, pLegs[i]->pBase->GetPmPtr(), 4, 0, &J[i * 3][0], 6);
			s_dgemm(3, 3, 3, -1, &J[i * 3][0], 6, *cm, 3, 0, &J[i * 3][3], 6);

			//calculate vJ
			s_dgemm(3, 3, 3, -1, *vLegJac, 3, *R, 3, 0, &vJ[i * 3][0], 6);
			s_dgemm(3, 3, 3, -1, *legJac, 3, *vR, 3, 1, &vJ[i * 3][0], 6);

			s_dgemm(3, 3, 3, -1, *vLegJac, 3, *R, 3, 0, *tem, 3);
			s_dgemm(3, 3, 3, -1, *tem, 3, *cm, 3, 0, &vJ[i * 3][3], 6);

			s_dgemm(3, 3, 3, -1, *legJac, 3, *vR, 3, 0, *tem, 3);
			s_dgemm(3, 3, 3, -1, *tem, 3, *cm, 3, 1, &vJ[i * 3][3], 6);

			s_dgemm(3, 3, 3, -1, *legJac, 3, *R, 3, 0, *tem, 3);
			s_dgemm(3, 3, 3, -1, *tem, 3, *vcm, 3, 1, &vJ[i * 3][3], 6);

		}

		/*计算支撑腿中位控电机部分的雅克比矩阵及其导数、输入等*/
		dim = 0;
		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->pSf->GetActive())
			{
				if (pLegs[i]->pM1->GetMode() == MOTION::POS_CONTROL)
				{
					memcpy(&Jp[dim][0], &J[i * 3][0], 6 * sizeof(double));
					memcpy(&vJp[dim][0], &vJ[i * 3][0], 6 * sizeof(double));
					vp[dim] = pLegs[i]->vl1;
					ap[dim] = aIn[i * 3];
					dim++;
				};
				if (pLegs[i]->pM2->GetMode() == MOTION::POS_CONTROL)
				{
					memcpy(&Jp[dim][0], &J[i * 3 + 1][0], 6 * sizeof(double));
					memcpy(&vJp[dim][0], &vJ[i * 3 + 1][0], 6 * sizeof(double));
					vp[dim] = pLegs[i]->vl2;
					ap[dim] = aIn[i * 3 + 1];
					dim++;
				};
				if (pLegs[i]->pM3->GetMode() == MOTION::POS_CONTROL)
				{
					memcpy(&Jp[dim][0], &J[i * 3 + 2][0], 6 * sizeof(double));
					memcpy(&vJp[dim][0], &vJ[i * 3 + 2][0], 6 * sizeof(double));
					vp[dim] = pLegs[i]->vl3;
					ap[dim] = aIn[i * 3 + 2];
					dim++;
				};
			}
		}

		/*使用部分雅克比矩阵计算身体加速度*/
		static double body_acc[6], body_vel[6];
		static double s[18], rcond = 0.0000001;
		static int rank;

		GetBodyVel(body_vel);
		memcpy(body_acc, ap, 6 * sizeof(double));

		s_dgemm(dim, 1, 6, -1, *vJp, 6, body_vel, 1, 1, body_acc, 1);
		s_dgelsd(dim, 6, 1, *Jp, 6, body_acc, 1, s, rcond, &rank);

		/*设置身体以及各腿加速度*/
		pBody->SetAcc(body_acc);
		
		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->pSf->GetActive())
			{
				pLegs[i]->SetAee(aee,"G");
			}
			else
			{
				pLegs[i]->SetAin(&aIn[i*3]);
			}
		}
	}

	void ROBOT::FastDynMtxInPrt()
	{
		static double Cb[6][6][36], Loc_C[36][36], k_L[6][36][4], H[6][18], h[18];
		
		static int supported_Leg_Num, supported_id[6];
		LEG *pLeg[6] = { pLF, pLM, pLR, pRF, pRM, pRR };

		static int ipiv[36];
		static double s[36];
		double rcond = 0.0000000001;
		int rank;

		memset(**Cb, 0, 6 * 6 * 36 * sizeof(double));
		memset(**k_L, 0, 6 * 36 * 4 * sizeof(double));
		memset(*H, 0, 6 * 18 * sizeof(double));
		memset(h, 0, 18 * sizeof(double));

		supported_Leg_Num = 0;
		memset(supported_id, 0, 6 * sizeof(int));


		pBody->UpdateInPrt();
		s_tmv_dot_vel(pBody->GetPrtTmvPtr(), pBody->GetAccPtr(), pBody->GetPrtAccPtr());
		/*更新cb并写入到h中，机身只有重力和惯性力*/
		s_daxpy(6, -1, pBody->GetPrtFgPtr(), 1, h, 1);
		s_daxpy(6, 1, pBody->GetPrtFvPtr(), 1, h, 1);
		s_dgemm(6, 1, 6, 1, pBody->GetPrtImPtr(), 6, pBody->GetPrtAccPtr(), 1, 1, h, 1);


		/*对每条腿操作*/
		for (int i = 0; i < 6; ++i)
		{
			pLeg[i]->FastDynMtxInPrt();

			/*更新Cb*/
			s_block_cpy(6, 4, pLeg[i]->pU1->GetPrtCstMtxIPtr(), 0, 0, 6, *(Cb[i]), 0, 0, 36);
			s_block_cpy(6, 4, pLeg[i]->pU2->GetPrtCstMtxIPtr(), 0, 0, 6, *(Cb[i]), 0, 4, 36);
			s_block_cpy(6, 4, pLeg[i]->pU3->GetPrtCstMtxIPtr(), 0, 0, 6, *(Cb[i]), 0, 8, 36);

			/*复制C与c_M*/
			memcpy(Loc_C, pLeg[i]->_C, 36 * 36 * sizeof(double));
			memcpy(*(k_L[i]), pLeg[i]->_c_M, 36 * 4 * sizeof(double));

			if (pLeg[i]->pSf->GetActive())
			{
				/*更新支撑腿数量与id*/
				supported_id[i] = supported_Leg_Num;
				supported_Leg_Num += 1;

				/*计算k_L*/

				s_dgesv(36, 4, *Loc_C, 36, ipiv, *(k_L[i]), 4);

				/*更新H，对于机身，只有U副跟腿连接，所以4*3=12列相乘即可*/
				s_dgemm(6, 3, 12, -1, *(Cb[i]), 36, &k_L[i][0][1], 4, 1, &H[0][supported_Leg_Num * 3 - 3], 18);
			}
			else
			{
				//dsp(*k_L[i], 36, 1, 0, 0, 4);
				//dsp(*Loc_C, 36, 6,0,0,36);
				/*计算k*/
				s_dgesv(36, 4, *Loc_C, 36, ipiv, *(k_L[i]), 4);

				
			}

			/*更新h，对于机身，只有U副跟腿连接，所以4*3=12列相乘即可*/
			s_dgemm(6, 1, 12, -1, *(Cb[i]), 36, *(k_L[i]), 4, 1, h, 1);
		}

		/*求解支撑腿的驱动力*/
		s_dgelsd(6, supported_Leg_Num * 3, 1, *H, 18, h, 1, s, rcond, &rank);

		for (int i = 0; i < 6; ++i)
		{
			if (pLeg[i]->pSf->GetActive())
			{
				memcpy(&result[i * 3], &h[supported_id[i] * 3], sizeof(double)* 3);
				pLeg[i]->pM1->SetF_m(&h[supported_id[i] * 3]);
				pLeg[i]->pM2->SetF_m(&h[supported_id[i] * 3+1]);
				pLeg[i]->pM3->SetF_m(&h[supported_id[i] * 3+2]);
			}
			else
			{
				result[i * 3] = k_L[i][33][0];
				result[i * 3 + 1] = k_L[i][34][0];
				result[i * 3 + 2] = k_L[i][35][0];

				pLeg[i]->pM1->SetF_m(&k_L[i][33][0]);
				pLeg[i]->pM2->SetF_m(&k_L[i][34][0]);
				pLeg[i]->pM3->SetF_m(&k_L[i][35][0]);
			}
		}

		/*更新驱动力*/

	}
	void ROBOT::FastDynEeForce(const double *fIn_in, double *fEE_out, const char *RelativeCoodinate)
	{
		pLF->FastDynEeForce(fIn_in, fEE_out, RelativeCoodinate);
		pLM->FastDynEeForce(fIn_in + 3, fEE_out + 3, RelativeCoodinate);
		pLR->FastDynEeForce(fIn_in + 6, fEE_out + 6, RelativeCoodinate);
		pRF->FastDynEeForce(fIn_in + 9, fEE_out + 9, RelativeCoodinate);
		pRM->FastDynEeForce(fIn_in + 12, fEE_out + 12, RelativeCoodinate);
		pRR->FastDynEeForce(fIn_in + 15, fEE_out + 15, RelativeCoodinate);
	}

	void ROBOT::Clb(const char* filename)
	{





	}

	void ROBOT::ClbLeg(const char* filename)
	{





	}

	/*void ROBOT::TestClb()
	{
		cout << "Reading data begin..." << endl;

		ifstream f_pin, f_vin, f_ain, f_fin;
		static double pin[1800][18], vin[1800][18], ain[1800][18], fin[1800][18];

		f_pin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\pos_for_test.txt");
		f_vin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\vel_for_test.txt");
		f_ain.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\acc_for_test.txt");
		f_fin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\fce_for_test.txt");

		int prow = 0, vrow = 0, arow = 0, frow = 0;
		while (f_pin >> pin[0][prow])
		{
			++prow;
		}
		while (f_vin >> vin[0][vrow])
		{
			++vrow;
		}
		while (f_ain >> ain[0][arow])
		{
			++arow;
		}
		while (f_fin >> fin[0][frow])
		{
			++frow;
		}
		prow = prow / 18;
		vrow = vrow / 18;
		arow = arow / 18;
		frow = frow / 18;

		f_pin.close();
		f_vin.close();
		f_ain.close();
		f_fin.close();

		cout << "Reading data finished" << endl;
		cout << "And data num is:" << prow << "  " << vrow << "  " << arow << "  " << frow << endl << endl;
		
		f_pin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\pos_for_clb.txt");
		f_vin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\vel_for_clb.txt");
		f_ain.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\acc_for_clb.txt");
		f_fin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\fce_for_clb.txt");
		
		
		double bodypos[6] = { 0, 0, 0, 0, 0, 0 };

		ofstream f_calibrated_fin;

		f_calibrated_fin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\calibrated_fce.txt");

		for (int i = 0; i < num; ++i)
		{
			SetPin(pin[i], bodypos);
			SetVinWithFixedFeet(vin[i]);
			SetAinWithFixedFeet(ain[i]);

			for (int j = 0; j < 18; ++j)
			{
				_Motion[j].SetF_m(&fin[i][j]);
				_Motion[j].SetA_m(&ain[i][j]);
			}

			DynPre();
			DynMtxInPrt();
			Dyn();

			for (int j = 0; j < 18; ++j)
			{
				if (_Motion[j].GetMode() == MOTION::FCE_CONTROL)
				{
					f_calibrated_fin << fin[i][j];
				}
				else
				{
					f_calibrated_fin << _Motion[j].GetF_mPtr()[0];
				}
				f_calibrated_fin << "   ";
			}
			f_calibrated_fin << endl;
		}
		f_calibrated_fin.close();
		cout << "test finished" << endl;
	}*/
	/*void ROBOT::TestClbEeFce()
	{
		cout << "Reading data begin..." << endl;

		ifstream f_pin, f_vin, f_ain, f_fin;
		static double pin[1800][18], vin[1800][18], ain[1800][18], fin[1800][18];

		f_pin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\pos_for_test.txt");
		f_vin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\vel_for_test.txt");
		f_ain.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\acc_for_test.txt");
		f_fin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\fce_for_test.txt");

		int prow = 0, vrow = 0, arow = 0, frow = 0;
		while (f_pin >> pin[0][prow])
		{
			++prow;
		}
		while (f_vin >> vin[0][vrow])
		{
			++vrow;
		}
		while (f_ain >> ain[0][arow])
		{
			++arow;
		}
		while (f_fin >> fin[0][frow])
		{
			++frow;
		}
		prow = prow / 18;
		vrow = vrow / 18;
		arow = arow / 18;
		frow = frow / 18;

		f_pin.close();
		f_vin.close();
		f_ain.close();
		f_fin.close();

		cout << "Reading data finished" << endl;
		cout << "And data num is:" << prow << "  " << vrow << "  " << arow << "  " << frow << endl << endl;

		f_pin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\pos_for_test.txt");
		f_vin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\vel_for_test.txt");
		f_ain.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\acc_for_test.txt");
		f_fin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\fce_for_test.txt");


		double bodypos[6] = { 0, 0, 0, 0, 0, 0 };
		double bodyvel[6] = { 0, 0, 0, 0, 0, 0 };
		double bodyacc[6] = { 0, 0, 0, 0, 0, 0 };

		ofstream f_calibrated_fin;

		f_calibrated_fin.open("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_experiment\\14.09.11 test\\calibrated_fce.txt");

		double fee[18];

		for (int i = 0; i < num; ++i)
		{
			SetPin(pin[i], bodypos);
			SetVin(vin[i], bodyvel);
			SetAin(ain[i], bodyacc);

			FastDynEeForce(fin[i],fee);


			for (int j = 0; j < 18; ++j)
			{
				f_calibrated_fin << fee[j];
				f_calibrated_fin << "   ";
			}
			f_calibrated_fin << endl;
		}
		f_calibrated_fin.close();
		cout << "test finished" << endl;
	}*/

	int ROBOT::MoveWithKinect(double* currentH, double *nextH, double *data)
	{
#define RATIO -5600000
		double locCurrentH[6],locNextH[6];

		locCurrentH[0] = currentH[2];
		locCurrentH[1] = currentH[5];
		locCurrentH[2] = currentH[4];
		locCurrentH[3] = currentH[1];
		locCurrentH[4] = currentH[0];
		locCurrentH[5] = currentH[3];

		locNextH[0] = nextH[2];
		locNextH[1] = nextH[5];
		locNextH[2] = nextH[4];
		locNextH[3] = nextH[1];
		locNextH[4] = nextH[0];
		locNextH[5] = nextH[3];
		
		
		
		memset(data, 0, sizeof(double)* 9000 * 18);
		
		double pEE[18],pIn[18];

		double h = -0.70;
		double d = -0.75;
		double alpha = PI / 3;

		double pEEo[] = 
		{ d*cos(alpha), h, d*sin(alpha), 
			d*cos(0), h, d*sin(0), 
			d*cos(-alpha), h, d*sin(-alpha), 
			d*cos(PI - alpha), h, d*sin(PI - alpha), 
			d*cos(PI), h, d*sin(PI), 
			d*cos(alpha - PI), h, d*sin(alpha - PI) };

		double bodyEp[6], bodyEpo[6];
		memset(bodyEpo, 0, sizeof(bodyEpo));
		

		double stepH = 0.2;
		double stepD = 0.375;

		int index = 0;

		for (int i = 0; i < 6; i++)
		{
			pEEo[i * 3 + 1] += locCurrentH[i];
		}

#pragma region 第一步
		/*第一步*/

		/*抬起加速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[1] = pEEo[1] + (stepH - locCurrentH[0]) / 2 * acc_even(750, i + 1);
			pEE[7] = pEEo[7] + (stepH - locCurrentH[2]) / 2 * acc_even(750, i + 1);
			pEE[13] = pEEo[13] + (stepH - locCurrentH[4]) / 2 * acc_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*抬起减速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[1] = pEEo[1] + (stepH - locCurrentH[0]) / 2 * dec_even(750, i + 1);
			pEE[7] = pEEo[7] + (stepH - locCurrentH[2]) / 2 * dec_even(750, i + 1);
			pEE[13] = pEEo[13] + (stepH - locCurrentH[4]) / 2 * dec_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*往前加速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[0] = pEEo[0] + stepD / 4 * acc_even(750, i + 1);
			pEE[6] = pEEo[6] + stepD / 4 * acc_even(750, i + 1);
			pEE[12] = pEEo[12] + stepD / 4 * acc_even(750, i + 1);
			bodyEp[3] = bodyEpo[3] + stepD / 8 * acc_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*往前减速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[0] = pEEo[0] + stepD / 4 * dec_even(750, i + 1);
			pEE[6] = pEEo[6] + stepD / 4 * dec_even(750, i + 1);
			pEE[12] = pEEo[12] + stepD / 4 * dec_even(750, i + 1);
			bodyEp[3] = bodyEpo[3] + stepD / 8 * dec_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*落下加速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[1] = pEEo[1] - (stepH - locNextH[0]) / 2 * acc_even(750, i + 1);
			pEE[7] = pEEo[7] - (stepH - locNextH[2]) / 2 * acc_even(750, i + 1);
			pEE[13] = pEEo[13] - (stepH - locNextH[4]) / 2 * acc_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*落下减速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[1] = pEEo[1] - (stepH - locNextH[0]) / 2 * dec_even(750, i + 1);
			pEE[7] = pEEo[7] - (stepH - locNextH[2]) / 2 * dec_even(750, i + 1);
			pEE[13] = pEEo[13] - (stepH - locNextH[4]) / 2 * dec_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));
#pragma endregion

#pragma region 第二步
		/*第一步*/

		/*抬起加速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[4] = pEEo[4] + (stepH - locCurrentH[1]) / 2 * acc_even(750, i + 1);
			pEE[10] = pEEo[10] + (stepH - locCurrentH[3]) / 2 * acc_even(750, i + 1);
			pEE[16] = pEEo[16] + (stepH - locCurrentH[5]) / 2 * acc_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*抬起减速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[4] = pEEo[4] + (stepH - locCurrentH[1]) / 2 * dec_even(750, i + 1);
			pEE[10] = pEEo[10] + (stepH - locCurrentH[3]) / 2 * dec_even(750, i + 1);
			pEE[16] = pEEo[16] + (stepH - locCurrentH[5]) / 2 * dec_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*往前加速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[3] = pEEo[3] + stepD / 4 * acc_even(750, i + 1);
			pEE[9] = pEEo[9] + stepD / 4 * acc_even(750, i + 1);
			pEE[15] = pEEo[15] + stepD / 4 * acc_even(750, i + 1);
			bodyEp[3] = bodyEpo[3] + stepD / 8 * acc_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*往前减速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[3] = pEEo[3] + stepD / 4 * dec_even(750, i + 1);
			pEE[9] = pEEo[9] + stepD / 4 * dec_even(750, i + 1);
			pEE[15] = pEEo[15] + stepD / 4 * dec_even(750, i + 1);
			bodyEp[3] = bodyEpo[3] + stepD / 8 * dec_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*落下加速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[4] = pEEo[4] - (stepH - locNextH[1]) / 2 * acc_even(750, i + 1);
			pEE[10] = pEEo[10] - (stepH - locNextH[3]) / 2 * acc_even(750, i + 1);
			pEE[16] = pEEo[16] - (stepH - locNextH[5]) / 2 * acc_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));

		/*落下减速*/
		memcpy(pEE, pEEo, sizeof(pEE));
		memcpy(bodyEp, bodyEpo, sizeof(bodyEpo));
		for (int i = 0; i < 750; i++)
		{
			pEE[4] = pEEo[4] - (stepH - locNextH[1]) / 2 * dec_even(750, i + 1);
			pEE[10] = pEEo[10] - (stepH - locNextH[3]) / 2 * dec_even(750, i + 1);
			pEE[16] = pEEo[16] - (stepH - locNextH[5]) / 2 * dec_even(750, i + 1);

			this->SetPee(pEE, bodyEp, "G");
			this->GetPin(pIn);
			s_daxpy(18, RATIO, pIn, 1, &data[(i + index) * 18], 1);
		}
		index += 750;
		memcpy(pEEo, pEE, sizeof(pEE));
		memcpy(bodyEpo, bodyEp, sizeof(bodyEp));
#pragma endregion



		return 0;
	}
}