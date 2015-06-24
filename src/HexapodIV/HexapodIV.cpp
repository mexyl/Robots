#ifndef ROBOT_EXPORTS
#define ROBOT_EXPORTS
#endif

#include "Platform.h"

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "HexapodIV.h"
#include <complex>
#include <cmath>
#include <ctime>
#include <iostream>

using namespace Aris::DynKer;
using namespace std;

namespace Robots
{	
	LEG_IV::LEG_IV(ROBOT_BASE* pRobot, unsigned beginPos)
		:LEG_BASE(pRobot, beginPos)
	{

	}

	/*以下为反解函数，你需要根据xyz和你自己定的尺寸参数来计算l1,l2,l3*/
	void LEG_IV::calculate_from_pEE()
	{
		
		
		l1 = sqrt(x*x + y*y + z*z - Sfy*Sfy - Sfz*Sfz) - Sfx;
		double b1 = asin(y / sqrt((l1 + Sfx)*(l1 + Sfx) + Sfy*Sfy)) - asin(Sfy / sqrt((l1 + Sfx)*(l1 + Sfx) + Sfy*Sfy));
		double a1 = atan2(Sfz*x - ((l1 + Sfx)*cos(b1) - Sfy*sin(b1))*z, ((l1 + Sfx)*cos(b1) - Sfy*sin(b1))*x + Sfz*z);
	
		double sa1 = sin(a1);
		double ca1 = cos(a1);
		double sb1 = sin(b1);
		double cb1 = cos(b1);

		double x2 = (l1 + S2x)*ca1*cb1 - S2y*ca1*sb1 + S2z*sa1 - U2x;
		double y2 = (l1 + S2x)*sb1 + S2y*cb1 + 0 - U2y;
		double z2 = -(l1 + S2x)*sa1*cb1 + S2y*sa1*sb1 + S2z*ca1 - U2z;

		double x3 = (l1 + S3x)*ca1*cb1 - S3y*ca1*sb1 + S3z*sa1 - U3x;
		double y3 = (l1 + S3x)*sb1 + S3y*cb1 + 0 - U3y;
		double z3 = -(l1 + S3x)*sa1*cb1 + S3y*sa1*sb1 + S3z*ca1 - U3z;

		l2 = sqrt(x2*x2 + y2*y2 + z2*z2);
		l3 = sqrt(x3*x3 + y3*y3 + z3*z3);
	}

	/*你需要修改这里，每条腿的坐标系相对于身体的欧拉角和位置*/
	const double ep0[6]{ PI / 2, PI * 5 / 6, -PI / 2 - PI * 7 / 18, -0.0679295, 0, -0.3552145 };
	const double ep1[6]{ PI / 2, PI * 6 / 6, -PI / 2 - PI * 7 / 18, -0.1234141, 0, 0 };
	const double ep2[6]{ PI / 2, PI * 7 / 6, -PI / 2 - PI * 7 / 18, -0.0679295, 0, 0.3532904 };
	const double ep3[6]{ PI / 2, PI * 1 / 6, -PI / 2 - PI * 7 / 18, 0.0679295, 0, -0.3552145 };
	const double ep4[6]{ PI / 2, PI * 0 / 6, -PI / 2 - PI * 7 / 18, 0.1234141, 0, 0 };
	const double ep5[6]{ PI / 2, PI * 11 / 6, -PI / 2 - PI * 7 / 18, 0.0679295, 0, 0.3532904 };


	ROBOT_IV::ROBOT_IV()
	{
		ROBOT_BASE::pLegs[0] = static_cast<LEG_BASE*>(&leg0);
		ROBOT_BASE::pLegs[1] = static_cast<LEG_BASE*>(&leg1);
		ROBOT_BASE::pLegs[2] = static_cast<LEG_BASE*>(&leg2);
		ROBOT_BASE::pLegs[3] = static_cast<LEG_BASE*>(&leg3);
		ROBOT_BASE::pLegs[4] = static_cast<LEG_BASE*>(&leg4);
		ROBOT_BASE::pLegs[5] = static_cast<LEG_BASE*>(&leg5);

		s_ep2pm(ep0, const_cast<double * const>(leg0.pBasePrtPm));
		s_ep2pm(ep1, const_cast<double * const>(leg1.pBasePrtPm));
		s_ep2pm(ep2, const_cast<double * const>(leg2.pBasePrtPm));
		s_ep2pm(ep3, const_cast<double * const>(leg3.pBasePrtPm));
		s_ep2pm(ep4, const_cast<double * const>(leg4.pBasePrtPm));
		s_ep2pm(ep5, const_cast<double * const>(leg5.pBasePrtPm));
	}
}