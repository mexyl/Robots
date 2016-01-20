#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
#ifdef UNIX
	Robots::SendRequest(argc, argv, "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
#ifdef WIN32
	Robots::SendRequest(argc, argv, "C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif

	return 0;
}
