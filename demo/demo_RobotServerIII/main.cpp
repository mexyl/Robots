
#include <Robot_Server.h>

int main()
{
	auto rs = Robots::RobotServer::GetInstance();

	//rs->CreateRobot<RobotTypeI>();

#ifdef UNIX
	rs->LoadXml("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
	rs->AddGait("wk", walk, parse);
	rs->Start();
#endif
#ifdef WIN32
	rs->LoadXml("C:\\Robots\\resource\\HexapodIII\\HexapodIII.xml");
	rs->AddGait("wk", Robots::walk, Robots::parseWalk);
#endif

	
	
	//
	/**/
	std::cout<<"finished"<<std::endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
