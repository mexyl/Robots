#include <aris.h>

int main(int argc, char *argv[])
{
#ifdef UNIX
	Aris::Server::sendRequest(argc, argv, "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
#ifdef WIN32
	Aris::Server::sendRequest(argc, argv, "C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif

	return 0;
}
