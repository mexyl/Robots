#ifndef ROBOT_IV_H
#define ROBOT_IV_H

#include <Aris_DynKer.h>
#include <fstream>

#include "Robot_Base.h"	



//需要修改默认的堆栈大小
//在windows里，位于程序属性->链接器->系统->堆栈保留大小


namespace Robots
{
	class LEG_IV:public Robots::LEG_BASE
	{
		friend class ROBOT_IV;
		
		/*你需要修改这里，这里是单腿你要用到的尺寸变量*/
		const double U2x{ 0 }, U2y{ 0.234 }, U2z{ 0.135 }, U3x{ 0 }, U3y{ 0.234 }, U3z{ -0.135 };
		const double S2x{ 0 }, S2y{ 0.059 }, S2z{ 0.034 }, S3x{ 0 }, S3y{ 0.059 }, S3z{ -0.034 };
		const double Sfx{ 0.1 }, Sfy{ 0 }, Sfz{ 0 };

	public:
		LEG_IV(ROBOT_BASE* pRobot, unsigned beginPos);
		~LEG_IV() = default;

		virtual void calculate_from_pEE();
	};

	class ROBOT_IV :public Robots::ROBOT_BASE
	{
		LEG_IV leg0{ this, 0 };
		LEG_IV leg1{ this, 3 };
		LEG_IV leg2{ this, 6 };
		LEG_IV leg3{ this, 9 };
		LEG_IV leg4{ this, 12 };
		LEG_IV leg5{ this, 15 };

	public:
		ROBOT_IV();
		~ROBOT_IV() = default;
	};

	
}

#endif