#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris_core.h>
#include <aris_message.h>
#include <aris_imu.h>
#include <aris_plan.h>
#include <aris_control_server.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

using namespace Aris::Core;

int main()
{
	auto &rs = Aris::ControlServer::Instance();
	rs.CreateRobot<Robots::RobotTypeI>();
#ifdef WIN32
	rs.LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	rs.LoadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
	//rs->AddGait("wk", Robots::walk, Robots::parseWalk);
	//rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	//rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
	//rs->AddGait("ro", Robots::resetOrigin, Robots::parseResetOrigin);
	rs.Start();
	std::cout<<"started"<<std::endl;

	
	
	Aris::Core::RunMsgLoop();

	return 0;
}
