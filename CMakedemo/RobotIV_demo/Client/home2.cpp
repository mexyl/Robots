#include <iostream>
#include <Aris_Core.h>
#include <Aris_Socket.h>

#include "../Server/robot_interface.h"

int main()
{
	Aris::Core::CONN c;
	c.Connect("127.0.0.1","5866");

	Aris::Core::MSG msg(EXECUTE_CMD);
	ROBOT_CMD cmd;
	cmd.id=HOME_2;
	msg.CopyStruct(cmd);

	c.SendRequest(msg);

	return 0;
}
