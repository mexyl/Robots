#ifndef ROBOT_EXPORTS
#define ROBOT_EXPORTS
#endif

#include "Platform.h"

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "HexapodIII.h"
#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

using namespace Aris::DynKer;
using namespace std;

namespace Robots
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
	
	LEG_III::LEG_III(const char *Name, ROBOT_III* pRobot)
		: OBJECT(static_cast<Aris::DynKer::MODEL *>(pRobot), Name)
		, pRobot(pRobot)
		, LEG_BASE(pRobot)
	{
	}

	void LEG_III::GetFin(double *fIn) const
	{
		for (unsigned i = 0; i < 3; ++i)
		{
			fIn[i] = *pMots[i]->GetF_mPtr();
		}
	}

	void LEG_III::calculate_from_pEE()
	{
		_CalCdByPos();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LEG_III::calculate_from_pIn()
	{
		_CalCdByPlen();
		_CalVarByCd();
		_CalPartByVar();
	}
	void LEG_III::calculate_from_vEE()
	{
		_CalVcdByVpos();
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LEG_III::calculate_from_vIn()
	{
		_CalVcdByVplen();
		_CalVvarByVcd();
		_CalVpartByVvar();
	}
	void LEG_III::calculate_from_aEE()
	{
		_CalAcdByApos();
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LEG_III::calculate_from_aIn()
	{
		_CalAcdByAplen();
		_CalAvarByAcd();
		_CalApartByAvar();
	}
	void LEG_III::calculate_jac()
	{
		int ipiv[3];
		double tem[3][3];

		/*direct*/
		memcpy(*_jac_vel_dir, *this->J1, sizeof(this->J1));
		memcpy(*tem, *this->J2, sizeof(this->J2));
		s_dgesvT(3, 3, *tem, 3, ipiv, *_jac_vel_dir, 3);

		/*inverse*/
		memcpy(*_jac_vel_inv, *this->J2, sizeof(this->J2));
		memcpy(*tem, *this->J1, sizeof(this->J1));
		s_dgesvT(3, 3, *tem, 3, ipiv, *_jac_vel_inv, 3);
	}
	void LEG_III::calculate_jac_c()
	{
		double tem[3];
		double vCD[3]{va1, vb1, vl1};

		/*direct*/
		s_dgemm(3, 1, 3, 1, *vJ2, 3, vCD, 1, 0, tem, 1);
		s_dgemm(3, 1, 3, -1, *_jac_vel_dir, 3, tem, 1, 0, _c_acc_dir, 1);
		s_dgemm(3, 1, 3, 1, *vJ1, 3, vCD, 1, 1, _c_acc_dir, 1);

		/*inverse*/
		s_dgemm(3, 1, 3, 1, *vJ1, 3, vCD, 1, 0, tem, 1);
		s_dgemm(3, 1, 3, -1, *_jac_vel_inv, 3, tem, 1, 0, _c_acc_inv, 1);
		s_dgemm(3, 1, 3, 1, *vJ2, 3, vCD, 1, 1, _c_acc_inv, 1);
	}

	void LEG_III::_CalCdByPos()
	{
		l1 = sqrt(x*x + y*y + z*z - Sfy*Sfy - Sfz*Sfz) - Sfx;
		b1 = asin(y / sqrt((l1 + Sfx)*(l1 + Sfx) + Sfy*Sfy)) - asin(Sfy / sqrt((l1 + Sfx)*(l1 + Sfx) + Sfy*Sfy));
		a1 = atan2(Sfz*x - ((l1 + Sfx)*cos(b1) - Sfy*sin(b1))*z, ((l1 + Sfx)*cos(b1) - Sfy*sin(b1))*x + Sfz*z);
	}
	void LEG_III::_CalCdByPlen()
	{
		static std::complex<double>  M, N;
		static std::complex<double>  K1, K2, K3;
		static std::complex<double>  p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
		static std::complex<double>  A, B, C;

		static double  X;

		M = (2 * (l1*l1 + H1*H1 + D1*D1 + H2*H2 + D2*D2) - l2*l2 - l3*l3) / 4;
		N = (l2*l2 - l3*l3) / 4;

		K1 = -2.0*M / (H1*sqrt(l1*l1 + H2*H2));
		K2 = (M*M - D1*D1*D2*D2) / (H1*H1*(l1*l1 + H2*H2)) - 1.0;
		K3 = -(M*M - D1*D1*D2*D2) / (H1*H1*(l1*l1 + H2*H2)) - D2*D2*N*N / (H1*H1*(l1*l1 + H2*H2)*(l1*l1 + H2*H2));

		//方程为：
		//x^4+K1*x^3+K2*x^2-K1*x+K3=0
		//solve 4th equation
		p1 = K1 + (K1*K2) / 2.0 - K1*K1*K1 / 8.0;
		p2 = p1*p1;
		p3 = K2 - 3.0*K1*K1 / 8.0;
		p4 = K3 + K1*K1*K2 / 16.0 + K1*K1 / 4.0 - (3.0*K1*K1*K1*K1) / 256.0;
		p5 = sqrt(4.0*p2*p3*p3*p3 - 16.0*p3*p3*p3*p3*p4 + 27.0*p2*p2 + 128.0*p3*p3*p4*p4 - 256.0*p4*p4*p4 - 144.0*p2*p3*p4);
		p6 = p3*p3*p3 / 27.0 + sqrt(3.0)*p5 / 18.0 + p2 / 2.0 - (4.0*p3*p4) / 3.0;
		p7 = pow(p6, 1.0 / 3.0);
		p8 = sqrt(9.0*p7*p7 + p3*p3 - 6.0*p3*p7 + 12.0*p4);
		p9 = 54.0*p1*sqrt(p6);
		p10 = -p3*p3*p8 - 12.0*p4*p8 - 12.0*p3*p7*p8 - 9.0*p7*p7*p8;

		A = p8 / (6.0*sqrt(p7));
		B = sqrt(p9 + p10) / (6.0*sqrt(p7*p8));
		C = sqrt(-p9 + p10) / (6.0*sqrt(p7*p8));

		//方程的四个根为：
		//root1 = -K1/4.0 + A + B;
		//root2 = -K1/4.0 + A - B;
		//root3 = -K1/4.0 - A + C;
		//root4 = -K1/4.0 - A - C;

		X = (-K1 / 4.0 - A + C).real();
		//solve finished

		b1 = asin(X) - atan(H2 / l1);
		a1 = asin(N.real() / (D1*(l1*cos(b1) - H2*sin(b1))));
	}
	void LEG_III::_CalCdByPlen2()
	{
		static double T1, T2, F1, F2, dq1, dq2, Ja, Jb, Jc, Jd;
		static int i;

		a1 = 0; b1 = 0;

		T1 = (U2x*U2x + U2y*U2y + U2z*U2z + (l1 + S2x)*(l1 + S2x) + S2y*S2y + S2z*S2z - l2*l2) / 2;
		T2 = (U3x*U3x + U3y*U3y + U3z*U3z + (l1 + S3x)*(l1 + S3x) + S3y*S3y + S3z*S3z - l3*l3) / 2;

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
	void LEG_III::_CalVarByCd()
	{
		sa1 = sin(a1);
		ca1 = cos(a1);
		sb1 = sin(b1);
		cb1 = cos(b1);

		x = (l1 + Sfx)*ca1*cb1 - Sfy*ca1*sb1 + Sfz*sa1;
		y = (l1 + Sfx)*sb1 + Sfy*cb1 + 0;
		z = -(l1 + Sfx)*sa1*cb1 + Sfy*sa1*sb1 + Sfz*ca1;

		x2 = (l1 + S2x)*ca1*cb1 - S2y*ca1*sb1 + S2z*sa1 - U2x;
		y2 = (l1 + S2x)*sb1 + S2y*cb1 + 0 - U2y;
		z2 = -(l1 + S2x)*sa1*cb1 + S2y*sa1*sb1 + S2z*ca1 - U2z;

		x3 = (l1 + S3x)*ca1*cb1 - S3y*ca1*sb1 + S3z*sa1 - U3x;
		y3 = (l1 + S3x)*sb1 + S3y*cb1 + 0 - U3y;
		z3 = -(l1 + S3x)*sa1*cb1 + S3y*sa1*sb1 + S3z*ca1 - U3z;

		l2 = sqrt(x2*x2 + y2*y2 + z2*z2);
		b2 = asin(y2 / l2);
		a2 = -atan(z2 / x2);

		l3 = sqrt(x3*x3 + y3*y3 + z3*z3);
		b3 = asin(y3 / l3);
		a3 = -atan(z3 / x3);

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
		J1[1][1] = x*ca1 - z*sa1;
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
	void LEG_III::_CalPartByVar()
	{
		double pm[4][4], pm1[4][4];

		pBase->Update();

		double pe[6] = { 0, 0, 0, PI / 2, a1, -PI / 2 + b1 };
		s_pe2pm(pe, *pm);
		s_pm_dot_pm(pBase->GetPmPtr(), *pm, *pm1);
		pP1a->SetPm(*pm1);

		double pe1[6] = { l1, 0, 0, 0, 0, 0 };
		s_pe2pm(pe1, *pm);
		s_pm_dot_pm(pP1a->GetPmPtr(), *pm, *pm1);
		pThigh->SetPm(*pm1);

		double pe2[6] = { U2x, U2y, U2z, PI / 2, a2, -PI / 2 + b2 };
		s_pe2pm(pe2, *pm);
		s_pm_dot_pm(pBase->GetPmPtr(), *pm, *pm1);
		pP2a->SetPm(*pm1);

		double pe3[6] = { l2, 0, 0, 0, 0, 0 };
		s_pe2pm(pe3, *pm);
		s_pm_dot_pm(pP2a->GetPmPtr(), *pm, *pm1);
		pP2b->SetPm(*pm1);

		double pe4[6] = { U3x, U3y, U3z, PI / 2, a3, -PI / 2 + b3 };
		s_pe2pm(pe4, *pm);
		s_pm_dot_pm(pBase->GetPmPtr(), *pm, *pm1);
		pP3a->SetPm(*pm1);

		double pe5[6] = { l3, 0, 0, 0, 0, 0 };
		s_pe2pm(pe5, *pm);
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
	void LEG_III::_CalVcdByVpos()
	{
		vl1 = (x*vx + y*vy + z*vz) / (l1 + Sfx);
		vb1 = (vy - vl1*sb1) / (x*ca1 - z*sa1);
		va1 = (vx*sa1 + vz*ca1) / (-x*ca1 + z*sa1);
	}
	void LEG_III::_CalVcdByVplen()
	{
		static double K1, K2;

		K1 = -l2*vl2 - k23*vl1;
		K2 = -l3*vl3 - k33*vl1;

		va1 = (k32*K1 - k22*K2) / (k21*k32 - k22*k31);
		vb1 = (-k31*K1 + k21*K2) / (k21*k32 - k22*k31);
		this->vl1 = vl1;
	}
	void LEG_III::_CalVvarByVcd()
	{
		vx = z*va1 - y*ca1*vb1 + ca1*cb1*vl1;
		vy = 0 + (x*ca1 - z*sa1)*vb1 + sb1*vl1;
		vz = -x*va1 + y*sa1*vb1 - sa1*cb1*vl1;

		vx2 = (z2 + U2z)*va1 - (y2 + U2y)*ca1*vb1 + ca1*cb1*vl1;
		vy2 = 0 + ((x2 + U2x)*ca1 - (z2 + U2z)*sa1)*vb1 + sb1*vl1;
		vz2 = -(x2 + U2x)*va1 + (y2 + U2y)*sa1*vb1 - sa1*cb1*vl1;

		vl2 = (x2*vx2 + y2*vy2 + z2*vz2) / l2;
		vb2 = (vy2 - vl2*sb2) / (l2*cb2);
		va2 = (vx2*sa2 + vz2*ca2) / (-x2*ca2 + z2*sa2);

		vx3 = (z3 + U3z)*va1 - (y3 + U3y)*ca1*vb1 + ca1*cb1*vl1;
		vy3 = 0 + ((x3 + U3x)*ca1 - (z3 + U3z)*sa1)*vb1 + sb1*vl1;
		vz3 = -(x3 + U3x)*va1 + (y3 + U3y)*sa1*vb1 - sa1*cb1*vl1;

		vl3 = (x3*vx3 + y3*vy3 + z3*vz3) / l3;
		vb3 = (vy3 - vl3*sb3) / (l3*cb3);
		va3 = (vx3*sa3 + vz3*ca3) / (-x3*ca3 + z3*sa3);

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
		vJ1[0][1] = -vy*ca1 - y*qa1;
		vJ1[0][2] = H22;
		vJ1[1][0] = 0;
		vJ1[1][1] = vx*ca1 + x*qa1 - vz*sa1 - z*pa1;
		vJ1[1][2] = pb1;
		vJ1[2][0] = -vx;
		vJ1[2][1] = vy*sa1 + y*pa1;
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
	void LEG_III::_CalVpartByVvar()
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
	void LEG_III::_CalAcdByApos()
	{
		static double xd, yd, zd;

		xd = ax - vJ1[0][0] * va1 - vJ1[0][1] * vb1 - vJ1[0][2] * vl1;
		yd = ay - vJ1[1][0] * va1 - vJ1[1][1] * vb1 - vJ1[1][2] * vl1;
		zd = az - vJ1[2][0] * va1 - vJ1[2][1] * vb1 - vJ1[2][2] * vl1;

		al1 = (x*xd + y*yd + z*zd) / (l1 + Sfx);
		ab1 = (yd - al1*sb1) / (x*ca1 - z*sa1);
		aa1 = (xd*sa1 + zd*ca1) / (-x*ca1 + z*sa1);
	}
	void LEG_III::_CalAcdByAplen()
	{
		static double vK1, vK2, M1, M2;

		vK1 = -vl2*vl2 - l2*al2 - vk23*vl1 - k23*al1;
		vK2 = -vl3*vl3 - l3*al3 - vk33*vl1 - k33*al1;

		M1 = vK1 - vk21*va1 - vk22*vb1;
		M2 = vK2 - vk31*va1 - vk32*vb1;

		aa1 = (k32*M1 - k22*M2) / (k21*k32 - k22*k31);
		ab1 = (-k31*M1 + k21*M2) / (k21*k32 - k22*k31);
	}
	void LEG_III::_CalAvarByAcd()
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
			- vx2*va1 + vy2*sa1*vb1 + (y2 + U2y)*pa1*vb1 - H12*vl1;

		al2 = (x2*ax2 + vx2*vx2 + y2*ay2 + vy2*vy2 + z2*az2 + vz2*vz2 - vl2*vl2)
			/ l2;
		ab2 = (ay2 - pb2*vl2 - sb2*al2 - vl2*pb2 - l2*qb2*vb2)
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
	void LEG_III::_CalApartByAvar()
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

	void LEG_III::FastDyn()
	{
		double rcond = 0.0000001;


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

	ROBOT_III::ROBOT_III()
		: pLF{ &LF_Leg }
		, pLM{ &LM_Leg }
		, pLR{ &LR_Leg }
		, pRF{ &RF_Leg }
		, pRM{ &RM_Leg }
		, pRR{ &RR_Leg }
	{
		for (unsigned int i = 0; i < 6; ++i)
		{
			Robots::ROBOT_BASE::pLegs[i] = static_cast<Robots::LEG_BASE *>(pLegs[i]);
		}
	}
	void ROBOT_III::GetFin(double *fIn) const
	{
		for (unsigned i = 0; i < 6; ++i)
		{
			pLegs[i]->GetFin(&fIn[i*3]);
		}
	}
	void ROBOT_III::SetFixedFeet(const char *fixedLeg, const char *ActiveMotion)
	{
		MOTION** const mots[18] =
		{ &(pLF->pM1), &(pLF->pM2), &(pLF->pM3),
		&(pLM->pM1), &(pLM->pM2), &(pLM->pM3),
		&(pLR->pM1), &(pLR->pM2), &(pLR->pM3),
		&(pRF->pM1), &(pRF->pM2), &(pRF->pM3),
		&(pRM->pM1), &(pRM->pM2), &(pRM->pM3),
		&(pRR->pM1), &(pRR->pM2), &(pRR->pM3), };

		int leg, Mot;

		if (fixedLeg == 0)
		{
			for (int i = 0; i < 6; ++i)
			{
				pLegs[i]->pSf->Activate();
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
	void ROBOT_III::FastDyn()
	{
		double Cb[6][6][36], Loc_C[36][36], k_L[6][36][4], H[6][18], h[18];

		int supported_Leg_Num, supported_id[6];

		int ipiv[36];
		double s[36];
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
			pLegs[i]->FastDyn();

			/*更新Cb*/
			s_block_cpy(6, 4, pLegs[i]->pU1->GetPrtCstMtxIPtr(), 0, 0, 6, *(Cb[i]), 0, 0, 36);
			s_block_cpy(6, 4, pLegs[i]->pU2->GetPrtCstMtxIPtr(), 0, 0, 6, *(Cb[i]), 0, 4, 36);
			s_block_cpy(6, 4, pLegs[i]->pU3->GetPrtCstMtxIPtr(), 0, 0, 6, *(Cb[i]), 0, 8, 36);

			/*复制C与c_M*/
			memcpy(Loc_C, pLegs[i]->_C, 36 * 36 * sizeof(double));
			memcpy(*(k_L[i]), pLegs[i]->_c_M, 36 * 4 * sizeof(double));

			if (pLegs[i]->pSf->GetActive())
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
				/*计算k*/
				s_dgesv(36, 4, *Loc_C, 36, ipiv, *(k_L[i]), 4);


			}

			/*更新h，对于机身，只有U副跟腿连接，所以4*3=12列相乘即可*/
			s_dgemm(6, 1, 12, -1, *(Cb[i]), 36, *(k_L[i]), 4, 1, h, 1);
		}

		/*求解支撑腿的驱动力*/
		if (supported_Leg_Num>0)
			s_dgelsd(6, supported_Leg_Num * 3, 1, *H, 18, h, 1, s, rcond, &rank);

		for (int i = 0; i < 6; ++i)
		{
			if (pLegs[i]->pSf->GetActive())
			{
				/*以下输入动力学计算，并补偿摩擦力*/
				double fce;
				for (unsigned j = 0; j < 3; ++j)
				{
					fce = h[supported_id[i] * 3 + j]
						+ s_sgn(pLegs[i]->pMots[j]->GetV_mPtr()[0]) * pLegs[i]->pMots[j]->GetFrcCoePtr()[0]
						+ pLegs[i]->pMots[j]->GetV_mPtr()[0] * pLegs[i]->pMots[j]->GetFrcCoePtr()[1]
						+ pLegs[i]->pMots[j]->GetA_mPtr()[0] * pLegs[i]->pMots[j]->GetFrcCoePtr()[2];
					pLegs[i]->pMots[j]->SetF_m(&fce);
				}
			}
			else
			{
				/*以下输入动力学计算，并补偿摩擦力*/
				double fce;
				for (unsigned j = 0; j < 3; ++j)
				{
					fce = k_L[i][33 + j][0]
						+ s_sgn(pLegs[i]->pMots[j]->GetV_mPtr()[0]) * pLegs[i]->pMots[j]->GetFrcCoePtr()[0]
						+ pLegs[i]->pMots[j]->GetV_mPtr()[0] * pLegs[i]->pMots[j]->GetFrcCoePtr()[1]
						+ pLegs[i]->pMots[j]->GetA_mPtr()[0] * pLegs[i]->pMots[j]->GetFrcCoePtr()[2];
					pLegs[i]->pMots[j]->SetF_m(&fce);
				}
			}
		}
	}
	void ROBOT_III::LoadXML(const char *filename)
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

		/* Update leg kinematic parameters */
		const_cast<double *&>(pBodyPm) = pBody->GetPmPtr();
		const_cast<double *&>(pBodyVel) = pBody->GetVelPtr();
		const_cast<double *&>(pBodyAcc) = pBody->GetAccPtr();
		
		for (auto *pLeg:this->pLegs)
		{
			const_cast<double *&>(pLeg->pBasePm) = const_cast<double *>(pLeg->pBase->GetPmPtr());
			const_cast<const double *&>(pLeg->pBasePrtPm) = pLeg->pBase->GetPrtPmPtr();
		}

	}

	void ROBOT_III::SimulateInverse(GAIT_FUNC fun, GAIT_PARAM_BASE *param)
	{
		unsigned totalCount = fun(this, param, 0) + 1;

		ForEachElement([totalCount](Aris::DynKer::ELEMENT *e)
		{
			e->SetResultSize(totalCount);
		});
	}
	void ROBOT_III::SimulateForwardByAdams(GAIT_FUNC fun, GAIT_PARAM_BASE *param, const char *adamsFile, SIMULATE_SCRIPT *pScript)
	{
		unsigned totalCount = fun(this, param, 0) + 1;
		unsigned akimaSize = totalCount / 10 + 1;//需加上0点

		std::vector<double> time(akimaSize);
		for (unsigned i = 0; i < akimaSize; ++i)
		{
			time.at(i) = i*0.01;
		}

		std::vector<std::vector<double> > motionPos(18);
		std::vector<std::vector<double> > motionFce(18);
		for (auto &mot : motionPos)
		{
			mot.resize(akimaSize);
		}

		/*设置起始点机器人驱动的位置*/
		this->SetPee(param->beginPee, param->beginBodyPE, "G");
		for (unsigned j = 0; j < 18; ++j)
		{
			motionPos.at(j).at(0) = this->GetMotion(j)->GetP_mPtr()[0];
		}

		/*设置其他时间节点上机器人驱动的位置*/
		for (unsigned i = 1; i < akimaSize; ++i)
		{
			fun(this, param, i * 10 - 1);

			for (unsigned j = 0; j < 18; ++j)
			{
				motionPos.at(j).at(i) = this->GetMotion(j)->GetP_mPtr()[0];
			}
		}

		/*设置驱动的位置akima函数*/
		for (unsigned i = 0; i < 18; ++i)
		{
			this->GetMotion(i)->SetPosAkimaCurve(akimaSize, time.data(), motionPos.at(i).data());
		}
		
		/*设置仿真脚本*/
		if (pScript)
		{
			for (auto &i : pScript->script)
			{
				for (auto &j : i.second.joints)
				{
					if (i.first == 0)
					{
						this->SetPee(param->beginPee, param->beginBodyPE, "G");
						s_pm2pe(j.first->GetMakJ()->GetPrtPmPtr(), j.second.peMakJ);
						j.second.isModifyMakJ = true;
					}
					else
					{
						fun(this, param, i.first);
						s_pm2pe(j.first->GetMakJ()->GetPrtPmPtr(), j.second.peMakJ);
						j.second.isModifyMakJ = true;
					}
				}
			}

			pScript->ScriptEndTime(totalCount);
		}


		/*将机器人当前位置设置到初始位置处*/
		this->SetPee(param->beginPee, param->beginBodyPE, "G");
		MODEL::SaveAdams(adamsFile, pScript);
	}
}