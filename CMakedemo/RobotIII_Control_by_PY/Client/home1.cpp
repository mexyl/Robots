#include <iostream>

#include <Aris_Core.h>
#include <Aris_Socket.h>



int main()
{
	Aris::Core::CONN c;
	c.Connect("127.0.0.1","5866");


	char a;
	std::cin >> a;

}
