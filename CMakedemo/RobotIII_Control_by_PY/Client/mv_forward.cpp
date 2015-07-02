#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>

#include <Aris_Core.h>
#include <Aris_Message.h>

using namespace Aris::Core;

#include "../Server/robot_interface.h"



int main()
{
	char RemoteIp[] = "127.0.0.1";

	/*for (unsigned i = 0; i < ROBOT_CMD_COUNT; ++i)
	{
		for (unsigned j = 0; j < ROBOT_STATE_COUNT; ++j)
		{
			if (ROBOT_STATE_MACHINE[i][j] != -1)
			{
				printCMD(i);
				std::cout << "->";
				printState(j);
				std::cout << " : ";
				printState(ROBOT_STATE_MACHINE[i][j]);
				std::cout << std::endl;

			}
		}
	}*/


	//init_interface();










	//close_interface();


	char a;
	std::cin >> a;

}
