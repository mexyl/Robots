#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

#include <stdlib.h>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>

using namespace Aris::Core;

#include "robot_interface.h"
#include "trajectory_generator.h"


extern int HEXBOT_HOME_OFFSETS_RESOLVER[18];


int copyClient()
{
	const int TASK_NAME_LEN =1024;

	std::int32_t count = 0;
	std::int32_t nIndex = 0;
	char path[TASK_NAME_LEN] = {0};
	char cParam[100] = {0};
	char *proName = path;
	std::int32_t tmp_len;

	pid_t pId = getpid();
	sprintf(cParam,"/proc/%d/exe",pId);
	count = readlink(cParam, path, TASK_NAME_LEN);

	if (count < 0 || count >= TASK_NAME_LEN)
    {
        throw std::logic_error("Current System Not Surport Proc.\n");
    }
	else
	{
		nIndex = count - 1;

		for( ; nIndex >= 0; nIndex--)
		{
			if( path[nIndex] == '/' )//筛选出进程名
		    {
				nIndex++;
				proName += nIndex;
				break;
		    }
		}
	}

	std::string pwd(path,nIndex);


	Aris::Core::DOCUMENT doc;

	/*get all cmd names*/
	if(doc.LoadFile("/usr/Robots/CMakeDemo/Robot_III/resource/client.xml")!=0)
		throw std::logic_error("failed to read client.xml");

	auto pCmds=doc.FirstChildElement("Commands");

	if(pCmds==nullptr)
		throw std::logic_error("invalid client.xml");

	for (auto pChild = pCmds->FirstChildElement();
		pChild != nullptr;
		pChild = pChild->NextSiblingElement())
	{
		std::string fullpath=std::string("cp ")+pwd+std::string("Client ")+pwd+pChild->Name();

		//cout<<fullpath<<endl;
		system(fullpath.c_str());

	}

	return 0;
};


int main()
{
	Aris::RT_CONTROL::CSysInitParameters initParam;

	initParam.motorNum=18;
	initParam.homeMode=-1;
	initParam.homeTorqueLimit=950;
	initParam.homeHighSpeed=280000;
	initParam.homeLowSpeed=40000;
	initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;

	Aris::RT_CONTROL::ACTUATION cs;

	cs.SetTrajectoryGenerator(tg);
	cs.SysInit(initParam);
	cs.SysInitCommunication();
	cs.SysStart();

	/**/

	copyClient();
	//system(cp )



	Aris::Core::CONN control_interface;
	control_interface.SetOnReceivedConnection([](Aris::Core::CONN *pConn,const char *pRemoteIP,int remotePort)
	{
		cout << "control client received:" << endl;
		cout << "    remote ip is:" << pRemoteIP << endl;
		cout << endl;
		return 0;
	});
	control_interface.SetOnReceiveRequest([&cs](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		cout<<"received request"<<endl;

		map<string,string> params;
		char content[500];
		string cmd;

		int32_t size=0;
		int32_t beginPos=0;

		msg.PasteStruct(size);
		beginPos+=4;
		msg.PasteAt(content,size,beginPos);
		cmd.assign(content);
		beginPos+=size;

		int32_t paramNum;
		msg.PasteAt(&paramNum,4,beginPos);
		beginPos+=4;

		for(int i=0;i<paramNum;++i)
		{
			string cmdd,param;


			msg.PasteAt(&size,4,beginPos);
			beginPos+=4;
			msg.PasteAt(content,size,beginPos);
			cmdd.assign(content);
			beginPos+=size;

			msg.PasteAt(&size,4,beginPos);
			beginPos+=4;
			msg.PasteAt(content,size,beginPos);
			param.assign(content);
			beginPos+=size;

			params.insert(make_pair(cmdd,param));
		}

		if(cmd=="en")
		{
			int allMotors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
			int leftMotors[9]={0,1,2,6,7,8,12,13,14};
			int rightMotors[9]={3,4,5,9,10,11,15,16,17};

			for(auto &i:params)
			{
				if(i.first=="all")
				{
					msg.CopyStruct(ENABLE,18);
					msg.CopyMore(allMotors,sizeof(allMotors));
				}
				else if(i.first=="left")
				{
					msg.CopyStruct(ENABLE,9);
					msg.CopyMore(leftMotors,sizeof(leftMotors));
				}
				else if(i.first=="right")
				{
					msg.CopyStruct(ENABLE,9);
					msg.CopyMore(rightMotors,sizeof(rightMotors));
				}
				else if(i.first=="motor")
				{
					int id = stoi(i.second);
					cout<<"motor id is:"<<id<<endl;
					msg.CopyStruct(ENABLE,1,id);
				}
			}
		}

		if(cmd=="ds")
		{
			int allMotors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
			int leftMotors[9]={0,1,2,6,7,8,12,13,14};
			int rightMotors[9]={3,4,5,9,10,11,15,16,17};

			for(auto &i:params)
			{

				if(i.first=="all")
				{
					msg.CopyStruct(DISABLE,18);
					msg.CopyMore(allMotors,sizeof(allMotors));
				}
				else if(i.first=="left")
				{
					msg.CopyStruct(DISABLE,9);
					msg.CopyMore(leftMotors,sizeof(leftMotors));
				}
				else if(i.first=="right")
				{
					msg.CopyStruct(DISABLE,9);
					msg.CopyMore(rightMotors,sizeof(rightMotors));
				}
				else if(i.first=="motor")
				{
					int id = stoi(i.second);
					cout<<"motor id is:"<<id<<endl;
					msg.CopyStruct(DISABLE,1,id);
				}
			}
		}

		if(cmd=="hm")
		{
			int allMotors[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
			int leftMotors[9]={0,1,2,6,7,8,12,13,14};
			int rightMotors[9]={3,4,5,9,10,11,15,16,17};

			for(auto &i:params)
			{

				if(i.first=="all")
				{
					msg.CopyStruct(HOME,18);
					msg.CopyMore(allMotors,sizeof(allMotors));
				}
				else if(i.first=="left")
				{
					msg.CopyStruct(HOME,9);
					msg.CopyMore(leftMotors,sizeof(leftMotors));
				}
				else if(i.first=="right")
				{
					msg.CopyStruct(HOME,9);
					msg.CopyMore(rightMotors,sizeof(rightMotors));
				}
				else if(i.first=="motor")
				{
					int id = stoi(i.second);
					cout<<"motor id is:"<<id<<endl;
					msg.CopyStruct(HOME,1,id);
				}
			}
		}

		cout<<cmd<<endl;
		for(auto &i:params)
		{
			cout<<i.first<<":"<<i.second<<endl;
		}


		msg.SetMsgID(0);
		cs.NRT_PostMsg(msg);

		return MSG();
	});
	control_interface.SetOnReceivedData([&cs](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
	{
		cout<<"received data"<<endl;


		cs.NRT_PostMsg(msg);
		return 0;
	});
	control_interface.SetOnLoseConnection([](Aris::Core::CONN *pConn)
	{
		cout << "control_interface lost" << endl;

		while(true)
		{
			try
			{
				pConn->StartServer("5866");
				break;
			}
			catch(CONN::START_SERVER_ERROR &e)
			{
				cout <<e.what()<<endl<<"will restart in 5s"<<endl;
				usleep(5000000);
			}
		}

		return 0;
	});


	/*start server*/
	while(true)
	{
		try
		{
			control_interface.StartServer("5866");
			break;
		}
		catch(CONN::START_SERVER_ERROR &e)
		{
			cout <<e.what()<<endl<<"will restart in 5s"<<endl;
			usleep(5000000);
		}
	}


	/**/
	cout<<"finished"<<endl;


	Aris::Core::RunMsgLoop();

	return 0;
}
