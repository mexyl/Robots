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

Aris::Sensor::IMU imu;

int test(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	auto data = imu.GetSensorData();
	rt_printf("%d : %f  %f  %f\n", data.Get().time, data.Get().ax, data.Get().ay, data.Get().az);
	
	auto *p = static_cast<const Robots::WALK_PARAM *>(pParam);
	return p->totalCount - p->count;
}

int main()
{
//	imu.Start();
//
//for(int i=0;i<50;++i)
//{
//
//auto data = imu.GetSensorData();
//std::cout<<data.Get().a<<"  "<<data.Get().b<<"  "<<data.Get().c<<std::endl;
//usleep(100000);
//}	
	


	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_TYPE_I>();

#ifdef PLATFORM_IS_LINUX
	rs->LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	rs->LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif

	rs->AddGait("wk", Robots::walk, Robots::parseWalk);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->Start();



	Aris::Core::RunMsgLoop();

	return 0;
}
