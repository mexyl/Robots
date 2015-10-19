#include <Platform.h>

#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#ifdef PLATFORM_IS_LINUX
#include <Aris_Control.h>
#endif
#ifdef PLATFORM_IS_WINDOWS
#define rt_printf printf
#endif



#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_IMU.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

using namespace Aris::Core;

//int test(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
//{
//	auto data = imu.GetSensorData();
//	rt_printf("%d : %f  %f  %f\n", data.Get().time, data.Get().ax, data.Get().ay, data.Get().az);
//	
//	auto *p = static_cast<const Robots::WALK_PARAM *>(pParam);
//	return p->totalCount - p->count;
//}

int main()
{
	std::cout<<1<<std::endl;
	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_TYPE_I>();
std::cout<<2<<std::endl;
#ifdef PLATFORM_IS_LINUX
	rs->LoadXml("/usr/Robots/resource/Robot_Type_I/Robot_VIII.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	rs->LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII.xml");
#endif
std::cout<<3<<std::endl;
	rs->AddGait("wk", Robots::walk, Robots::parseWalk);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
	rs->Start();
std::cout<<4<<std::endl;
	Aris::Core::RunMsgLoop();

	return 0;
}
