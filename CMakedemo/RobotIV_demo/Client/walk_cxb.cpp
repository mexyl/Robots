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
	cmd.id=WALK_CXB;

	cmd.param[0].toInt=1;
	cmd.param[1].toDouble=100;
	cmd.param[2].toDouble=0;
	cmd.param[3].toDouble=0;
	cmd.param[4].toDouble=0.6;
	cmd.param[5].toDouble=60;
	cmd.param[6].toDouble=1;
	cmd.param[7].toDouble=650;



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

			if((t<0)||(t>10))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[0].toInt=t;
			i++;

			continue;
		}

		if(strcmp(argv[i],"-s")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double s = atof(argv[i+1]);

			if((s<0)||(s>300))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[1].toDouble=atof(argv[i+1]);
			i++;

			continue;
		}
		if(strcmp(argv[i],"-L")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double L = atof(argv[i+1]);

			if((L<-50)||(L>50))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[2].toDouble=L;
			i++;

			continue;
		}
		if(strcmp(argv[i],"-r")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double r = atof(argv[i+1]);

			if((r<-5)||(r>5))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[3].toDouble=r;
			i++;

			continue;
		}
		if(strcmp(argv[i],"-d")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double beta = atof(argv[i+1]);

			if((beta<0)||(beta>1))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[4].toDouble=beta;
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

			double h=atof(argv[i+1]);

			if((h<0)||(h>200))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[5].toDouble=h;
			i++;

			continue;
		}

		if(strcmp(argv[i],"-T")==0)
		{
			if(i+1>=argc)
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			double T=atof(argv[i+1]);

			if((T<0.8)||(T>5))
			{
				cout<<"invalid param "<<argv[i]<<endl;
				return -1;
			}

			cmd.param[6].toDouble=T;
			i++;

			continue;
		}

		if(strcmp(argv[i],"-H")==0)
				{
					if(i+1>=argc)
					{
						cout<<"invalid param "<<argv[i]<<endl;
						return -1;
					}

					double H=atof(argv[i+1]);

					if((H<500)||(H>1000))
					{
						cout<<"invalid param "<<argv[i]<<endl;
						return -1;
					}

					cmd.param[7].toDouble=H;
					i++;

					continue;
				}




		cout <<"invalid params:"<<argv[i]<<endl;
		return -1;
	}

	cout<<"forward:"<<endl;
	cout<<"    totalPeriod:   "<<cmd.param[0].toInt<<endl;
	cout<<"    stepLength :   "<<cmd.param[1].toDouble<<endl;
	cout<<"    Lside:         "<<cmd.param[2].toDouble<<endl;
	cout<<"    rotationAngle: "<<cmd.param[3].toDouble<<endl;
	cout<<"    duty:          "<<cmd.param[4].toDouble<<endl;
	cout<<"    stepheight:    "<<cmd.param[5].toDouble<<endl;
	cout<<"    T:             "<<cmd.param[6].toDouble<<endl;
	cout<<"    standHeight:   "<<cmd.param[7].toDouble<<endl;

	Aris::Core::MSG msg(EXECUTE_CMD);
	msg.CopyStruct(cmd);

	Aris::Core::CONN c;
	c.Connect("127.0.0.1","5866");
	c.SendRequest(msg);

	return 0;
}
