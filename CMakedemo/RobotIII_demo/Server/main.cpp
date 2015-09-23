#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#include <Platform.h>
#include <Aris_Control.h>
#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_IMU.h>
#include <Robot_Server.h>
#include <HexapodIII.h>

using namespace Aris::Core;

Aris::Sensor::IMU imu;

int test(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
	auto data = imu.GetSensorData();
	
	rt_printf("%d  %d  %d\n", data.Get().a, data.Get().b, data.Get().c);

}

int main()
{
	Aris::Sensor::IMU imu;
	imu.Start();
	
	
	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_III>();

#ifdef PLATFORM_IS_LINUX
	rs->LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	rs->LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
#endif

	rs->AddGait("wk", Robots::walk, test);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->Start();

	Aris::Core::RunMsgLoop();

	return 0;
}
