#include <iostream>
#include <Aris_Core.h>
#include <Aris_Socket.h>

#include "../Server/robot_interface.h"

#include <cstring>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char *argv[])
{
	ROBOT_CMD cmd;
	cmd.id=MV_FORWARD;

	cmd.param[0].toInt=1000;
	strcpy(cmd.param[1].toChar,"x");
	strcpy(cmd.param[2].toChar,"z");
	cmd.param[3].toDouble=0.2;
	cmd.param[4].toDouble=0.05;
	cmd.param[5].toInt=1;



	for (int i=1;i<argc;++i)
	{
		if(strcmp(argv[i],"-t")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			int t=atoi(argv[i+1]);

			if((t<500)||(t>5000))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[0].toInt=t;
			i++;

			continue;
		}


		if(strcmp(argv[i],"x")==0)
		{
			strcpy(cmd.param[1].toChar,"x");
			continue;
		}

		if(strcmp(argv[i],"-x")==0)
		{
			strcpy(cmd.param[1].toChar,"-x");
			continue;
		}

		if(strcmp(argv[i],"y")==0)
		{
			strcpy(cmd.param[1].toChar,"y");
			continue;
		}

		if(strcmp(argv[i],"-y")==0)
		{
			strcpy(cmd.param[1].toChar,"-y");
			continue;
		}


		if(strcmp(argv[i],"-d")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double d = atof(argv[i+1]);

			if((d<0)||(d>0.6))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[3].toDouble=atof(argv[i+1]);
			i++;

			continue;
		}
		if(strcmp(argv[i],"-h")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double h = atof(argv[i+1]);

			if((h<0)||(h>0.3))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[4].toDouble=h;
			i++;

			continue;
		}



		if(strcmp(argv[i],"-n")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			int n=atoi(argv[i+1]);

			if((n<1)||(n>9))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[5].toInt=n;
			i++;

			continue;
		}



		cout <<"invalid params:"<<argv[i]<<endl;
		return -1;
	}

	cout<<"forward:"<<endl;
	cout<<"    total count:   "<<cmd.param[0].toInt<<endl;
	cout<<"    walk direction:"<<cmd.param[1].toChar<<endl;
	cout<<"    up direction:  "<<cmd.param[2].toChar<<endl;
	cout<<"    step d:        "<<cmd.param[3].toDouble<<endl;
	cout<<"    step h:        "<<cmd.param[4].toDouble<<endl;
	cout<<"    step n:        "<<cmd.param[5].toInt<<endl;

	Aris::Core::MSG msg(EXECUTE_CMD);
	msg.CopyStruct(cmd);

	Aris::Core::CONN c;
	c.Connect("127.0.0.1","5866");
	c.SendRequest(msg);

	return 0;
}
