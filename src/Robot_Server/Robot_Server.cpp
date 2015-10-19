#include <Platform.h>
#ifdef PLATFORM_IS_WINDOWS
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif

#include <cstring>

#ifdef PLATFORM_IS_LINUX
#include <Aris_Control.h>
#endif

#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_ExpCal.h>
#include <Aris_Plan.h>
#include <Aris_Motion.h>
#include "Robot_Base.h"
#include "Robot_Server.h"

namespace Robots
{
	class NODE
	{
	public:
		NODE* AddChildGroup(const char *Name);
		NODE* AddChildUnique(const char *Name);
		NODE* AddChildParam(const char *Name);
		NODE* FindChild(const char *Name)
		{
			auto result = std::find_if(children.begin(), children.end(), [Name](std::unique_ptr<NODE> &node)
			{
				return (!std::strcmp(node->name.c_str(), Name));
			});

			if (result != children.end())
			{
				return result->get();
			}
			else
			{
				return nullptr;
			}
		};

		bool IsTaken() { return isTaken; };
		void Take();
		void Reset()
		{
			this->isTaken = false;
			for (auto &child : children)
			{
				child->Reset();
			}
		};

	public:
		NODE(NODE*father, const char *Name) :name(Name) { this->father = father; };
		virtual ~NODE() {};

	private:
		std::string name;
		NODE* father;
		std::vector<std::unique_ptr<NODE> > children;

		bool isTaken{ false };

		friend void addAllParams(const Aris::Core::ELEMENT *pEle, NODE *pNode, std::map<std::string, NODE *> &allParams, std::map<char, std::string>& shortNames);
		friend void addAllDefault(NODE *pNode, std::map<std::string, std::string> &params);
	};
	class ROOT_NODE :public NODE
	{
	public:
		ROOT_NODE(const char *Name) :NODE(nullptr, Name) {};

	private:
		NODE *pDefault;

		friend void addAllParams(const Aris::Core::ELEMENT *pEle, NODE *pNode, std::map<std::string, NODE *> &allParams, std::map<char, std::string>& shortNames);
		friend void addAllDefault(NODE *pNode, std::map<std::string, std::string> &params);
	};
	class GROUP_NODE :public NODE
	{
	public:
		GROUP_NODE(NODE*father, const char *Name) :NODE(father, Name) {};
	};
	class UNIQUE_NODE :public NODE
	{
	public:
		UNIQUE_NODE(NODE*father, const char *Name) :NODE(father, Name) {};

	private:
		NODE *pDefault;

		friend void addAllParams(const Aris::Core::ELEMENT *pEle, NODE *pNode, std::map<std::string, NODE *> &allParams, std::map<char, std::string>& shortNames);
		friend void addAllDefault(NODE *pNode, std::map<std::string, std::string> &params);
	};
	class PARAM_NODE :public NODE
	{
	public:
		PARAM_NODE(NODE*father, const char *Name) :NODE(father, Name) {};
	private:
		std::string type;
		std::string defaultValue;
		std::string minValue, maxValue;

		friend void addAllParams(const Aris::Core::ELEMENT *pEle, NODE *pNode, std::map<std::string, NODE *> &allParams, std::map<char, std::string>& shortNames);
		friend void addAllDefault(NODE *pNode, std::map<std::string, std::string> &params);
	};

	NODE* NODE::AddChildGroup(const char *Name)
	{
		this->children.push_back(std::unique_ptr<NODE>(new GROUP_NODE(this, Name)));
		return children.back().get();
	};
	NODE* NODE::AddChildUnique(const char *Name)
	{
		this->children.push_back(std::unique_ptr<NODE>(new UNIQUE_NODE(this, Name)));
		return children.back().get();
	}
	NODE* NODE::AddChildParam(const char *Name)
	{
		this->children.push_back(std::unique_ptr<NODE>(new PARAM_NODE(this, Name)));
		return children.back().get();
	}
	void NODE::Take()
	{
		if (dynamic_cast<ROOT_NODE*>(this))
		{
			if (this->isTaken)
			{
				throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
			}
			else
			{
				this->isTaken = true;
				return;
			}
		}
		else if (dynamic_cast<GROUP_NODE*>(this))
		{
			if (this->isTaken)
			{
				return;
			}
			else
			{
				this->isTaken = true;
				father->Take();
			}
		}
		else
		{
			if (this->isTaken)
			{
				throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
			}
			else
			{
				this->isTaken = true;
				father->Take();
			}
		}
	}

	void addAllParams(const Aris::Core::ELEMENT *pEle, NODE *pNode, std::map<std::string, NODE *> &allParams, std::map<char, std::string>& shortNames)
	{
		/*add all children*/
		for (auto pChild = pEle->FirstChildElement();pChild != nullptr;	pChild = pChild->NextSiblingElement())
		{
			/*check if children already has this value*/
			if (pNode->FindChild(pChild->Name()))
			{
				throw std::logic_error(std::string("XML file has error: node \"") + pChild->Name() + "\" already exist");
			}

			/*set all children*/
			if (pChild->Attribute("type", "group"))
			{
				addAllParams(pChild, pNode->AddChildGroup(pChild->Name()), allParams, shortNames);
			}
			else if (pChild->Attribute("type", "unique"))
			{
				addAllParams(pChild, pNode->AddChildUnique(pChild->Name()), allParams, shortNames);
			}
			else
			{
				/*now the pChild is a param_node*/
				NODE * insertNode;

				if (allParams.find(std::string(pChild->Name())) != allParams.end())
				{
					throw std::logic_error(std::string("XML file has error: node \"") + pChild->Name() + "\" already exist");
				}
				else
				{
					insertNode = pNode->AddChildParam(pChild->Name());
					allParams.insert(std::pair<std::string, NODE *>(std::string(pChild->Name()), insertNode));
				}

				/*set abbreviation*/
				if (pChild->Attribute("abbreviation"))
				{
					if (shortNames.find(*pChild->Attribute("abbreviation")) != shortNames.end())
					{
						throw std::logic_error(std::string("XML file has error: abbreviations \"")+ pChild->Attribute("abbreviation") + "\" already exist");
					}
					else
					{
						char abbr = *pChild->Attribute("abbreviation");
						shortNames.insert(std::pair<char, std::string>(abbr, std::string(pChild->Name())));
					}
				}

				/*set values*/
				if (pChild->Attribute("type"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->type = std::string(pChild->Attribute("type"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->type = "";
				}

				if (pChild->Attribute("default"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->defaultValue = std::string(pChild->Attribute("default"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->defaultValue = "";
				}

				if (pChild->Attribute("maxValue"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->maxValue = std::string(pChild->Attribute("maxValue"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->maxValue = "";
				}

				if (pChild->Attribute("minValue"))
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->minValue = std::string(pChild->Attribute("minValue"));
				}
				else
				{
					dynamic_cast<PARAM_NODE*>(insertNode)->minValue = "";
				}
			}
		}

		/*set all values*/
		if (dynamic_cast<ROOT_NODE*>(pNode))
		{
			if (pEle->Attribute("default"))
			{
				if (pNode->FindChild(pEle->Attribute("default")))
				{
					dynamic_cast<ROOT_NODE*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
				}
				else
				{
					throw std::logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
				}
			}
			else
			{
				dynamic_cast<ROOT_NODE*>(pNode)->pDefault = nullptr;
			}
		}

		if (dynamic_cast<UNIQUE_NODE*>(pNode))
		{
			if (pEle->Attribute("default"))
			{
				if (pNode->FindChild(pEle->Attribute("default")))
				{
					dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
				}
				else
				{
					throw std::logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
				}
			}
			else
			{
				if (pNode->children.empty())
				{
					throw std::logic_error(std::string("XML file has error: unique node \"") + pNode->name + "\" must have more than 1 child");
				}
				else
				{
					dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault = nullptr;
				}
			}
		}
	}
	void addAllDefault(NODE *pNode, std::map<std::string, std::string> &params)
	{
		if (pNode->isTaken)
		{
			if (dynamic_cast<ROOT_NODE*>(pNode))
			{
				auto found = find_if(pNode->children.begin(), pNode->children.end(), [](std::unique_ptr<NODE> &a)
				{
					return a->isTaken;
				});

				addAllDefault(found->get(), params);
			}

			if (dynamic_cast<UNIQUE_NODE*>(pNode))
			{
				auto found = find_if(pNode->children.begin(), pNode->children.end(), [](std::unique_ptr<NODE> &a)
				{
					return a->isTaken;
				});

				addAllDefault(found->get(), params);
			}

			if (dynamic_cast<GROUP_NODE*>(pNode))
			{
				for (auto &i : pNode->children)
					addAllDefault(i.get(), params);
			}

			if (dynamic_cast<PARAM_NODE*>(pNode))
			{
				if (params.at(pNode->name) == "")
				{
					params.at(pNode->name) = dynamic_cast<PARAM_NODE*>(pNode)->defaultValue;
				}
				return;
			}
		}
		else
		{
			if (dynamic_cast<ROOT_NODE*>(pNode))
			{
				if (!pNode->children.empty())
				{
					if ((dynamic_cast<ROOT_NODE*>(pNode)->pDefault))
					{
						addAllDefault(dynamic_cast<ROOT_NODE*>(pNode)->pDefault, params);
					}
					else
					{
						throw std::logic_error(std::string("cmd \"") + pNode->name + "\" has no default param");
					}
				}

				pNode->isTaken = true;
			}

			if (dynamic_cast<UNIQUE_NODE*>(pNode))
			{
				if (!pNode->children.empty())
				{
					if (dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault)
					{
						addAllDefault(dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault, params);
					}
					else
					{
						throw std::logic_error(std::string("param \"") + pNode->name + "\" has no default sub-param");
					}
				}

				pNode->isTaken = true;
			}

			if (dynamic_cast<GROUP_NODE*>(pNode))
			{
				for (auto &i : pNode->children)
				{
					addAllDefault(i.get(), params);
				}
					

				pNode->isTaken = true;
			}

			if (dynamic_cast<PARAM_NODE*>(pNode))
			{
				params.insert(make_pair(pNode->name, dynamic_cast<PARAM_NODE*>(pNode)->defaultValue));
				pNode->isTaken = true;
			}
		}
	}

	struct COMMAND_STRUCT
	{
		std::unique_ptr<ROOT_NODE> root;
		std::map<std::string, NODE *> allParams{};
		std::map<char, std::string> shortNames{};

		COMMAND_STRUCT(const std::string &name) 
			:root(new ROOT_NODE(name.c_str()))
		{
		}
	};
	
	class ROBOT_SERVER::IMP
	{
	public:
		void LoadXml(const Aris::Core::DOCUMENT &doc);
		void AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc);
		void Start();
		void Stop();

		IMP(ROBOT_SERVER *pServer) 
		{ 
			this->pServer = pServer;
#ifdef PLATFORM_IS_LINUX
			this->pController = Aris::Control::CONTROLLER::CreateMaster<Aris::Control::CONTROLLER>();
#endif
		};
	private:
		IMP(const IMP&) = delete;

		void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string, std::string> &params);
		void GenerateCmdMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::MSG &msg);
		void OnReceiveMsg(const Aris::Core::MSG &m, Aris::Core::MSG &retError);

		inline int p2a(const int phy)
		{
			return mapPhy2Abs[phy];
		}
		inline int a2p(const int abs)
		{
			return mapAbs2Phy[abs];
		}
		inline void p2a(const int *phy, int *abs, int num = 18)
		{
			for (int i = 0; i<num; ++i)
			{
				abs[i] = mapPhy2Abs[phy[i]];
			}
		}
		inline void a2p(const int *abs, int *phy, int num = 18)
		{
			for (int i = 0; i<num; ++i)
			{
				phy[i] = mapAbs2Phy[abs[i]];
			}
		}

		int home(const Robots::BASIC_FUNCTION_PARAM *pParam, Aris::Control::CONTROLLER::DATA data);
		int enable(const Robots::BASIC_FUNCTION_PARAM *pParam, Aris::Control::CONTROLLER::DATA data);
		int disable(const Robots::BASIC_FUNCTION_PARAM *pParam, Aris::Control::CONTROLLER::DATA data);
		int recover(Robots::RECOVER_PARAM *pParam, Aris::Control::CONTROLLER::DATA data);
		int runGait(Robots::GAIT_PARAM_BASE *pParam, Aris::Control::CONTROLLER::DATA data);

		int execute_cmd(int count, char *cmd, Aris::Control::CONTROLLER::DATA data);
		static int tg(Aris::Control::CONTROLLER::DATA &data);

	private:
		enum ROBOT_CMD_ID
		{
			ENABLE,
			DISABLE,
			HOME,
			RECOVER,
			RUN_GAIT,

			ROBOT_CMD_COUNT
		};

	private:
		ROBOT_SERVER *pServer;
		std::map<std::string, int> mapName2ID;//store gait id in follow vector
		std::vector<GAIT_FUNC> allGaits;
		std::vector<PARSE_FUNC> allParsers;

		std::map<std::string, std::unique_ptr<COMMAND_STRUCT> > mapCmd;//store NODE of command

		Aris::Core::CONN server;
		std::string ip, port;

		double alignEE[18], alignIn[18], recoverEE[18];
		double meter2count{ 0 };

		int mapPhy2Abs[18];
		int mapAbs2Phy[18];

//#ifdef PLATFORM_IS_LINUX
		Aris::Control::CONTROLLER *pController;
//#endif
		std::unique_ptr<Aris::Sensor::IMU> pImu;
		friend class ROBOT_SERVER;
	};

	void ROBOT_SERVER::IMP::LoadXml(const Aris::Core::DOCUMENT &doc)
	{
		/*load robot model*/
		pServer->pRobot->LoadXml(doc);
		
		/*begin to create imu*/
		if (doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU")->Attribute("Active", "true"))
		{
			pImu.reset(new Aris::Sensor::IMU(doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU")));
		}

		/*begin to load controller*/
#ifdef PLATFORM_IS_LINUX
		pController->LoadXml(doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Control")->FirstChildElement("EtherCat"));
		pController->SetControlStrategy(tg);
#endif

		/*load connection param*/
		auto pConnEle = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection");
		ip = pConnEle->Attribute("IP");
		port = pConnEle->Attribute("Port");

		/*motion parameter*/
		auto pContEle = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Control");
		Aris::DynKer::CALCULATOR c;
		meter2count = c.CalculateExpression(pContEle->Attribute("meter2count")).ToDouble();

		/*construct mapPhy2Abs and mapAbs2Phy*/
		std::string mapPhy2AbsText{ pContEle->FirstChildElement("MapPhy2Abs")->GetText() };
		std::stringstream stream(mapPhy2AbsText);
		for (int i = 0; i < 18; ++i)
		{
			std::string word;

			while (stream >> word)
			{
				std::stringstream intStream{ word };
				if (intStream >> mapPhy2Abs[i])
					break;
			}
		}
		for (int i = 0; i < 18; ++i)
		{
			mapAbs2Phy[i] = std::find(mapPhy2Abs, mapPhy2Abs + 18, i) - mapPhy2Abs;
		}


		/*load recover parameter*/
		auto mat = c.CalculateExpression(pContEle->FirstChildElement("AlignPee")->GetText());
		std::copy_n(mat.Data(), 18, alignEE);
		mat = c.CalculateExpression(pContEle->FirstChildElement("RecoverPee")->GetText());
		std::copy_n(mat.Data(), 18, recoverEE);
		double pe[6]{ 0 };
		pServer->pRobot->SetBodyPe(pe);
		pServer->pRobot->SetPee(alignEE);
		pServer->pRobot->GetPin(alignIn);

		/*set home offset*/
		mat = c.CalculateExpression(pContEle->FirstChildElement("homePin")->GetText());
		for (int i = 0; i < 18; ++i)
		{
#ifdef PLATFORM_IS_LINUX
			pController->Motion(i)->WriteSdo(9, static_cast<std::int32_t>(-mat.Data()[a2p(i)] * meter2count));
#endif
		}

		



		

		/*begin to copy client and insert cmd nodes*/
		const int TASK_NAME_LEN = 1024;
		char path_char[TASK_NAME_LEN] = { 0 };
#ifdef PLATFORM_IS_WINDOWS
		GetModuleFileName(NULL, path_char, TASK_NAME_LEN);
		std::string path(path_char);
		path = path.substr(0, path.rfind('\\'));
#endif
#ifdef PLATFORM_IS_LINUX
		char cParam[100] = { 0 };
		sprintf(cParam, "/proc/%d/exe", getpid());
		auto count = readlink(cParam, path_char, TASK_NAME_LEN);
		std::string path(path_char);
		path = path.substr(0, path.rfind('/'));
#endif

		auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

		if (pCmds == nullptr) throw std::logic_error("invalid xml file, because it contains no commands information");

		mapCmd.clear();
		for (auto pChild = pCmds->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
		{
#ifdef PLATFORM_IS_WINDOWS
			std::string fullpath = std::string("copy ") + path + std::string("\\Client.exe ") + path + "\\" + pChild->Name() + ".exe";
#endif
#ifdef PLATFORM_IS_LINUX
			std::string fullpath = std::string("cp ") + path + std::string("/Client ") + path + "/" + pChild->Name();
#endif
			auto ret = system(fullpath.c_str());

			if (mapCmd.find(pChild->Name())!=mapCmd.end())
				throw std::logic_error(std::string("command ")+ pChild->Name() +" is already existed, please rename it");

			mapCmd.insert(std::make_pair(std::string(pChild->Name()), std::unique_ptr<COMMAND_STRUCT>(new COMMAND_STRUCT(pChild->Name()))));
			addAllParams(pChild, mapCmd.at(pChild->Name())->root.get(), mapCmd.at(pChild->Name())->allParams, mapCmd.at(pChild->Name())->shortNames);
		}


	}
	void ROBOT_SERVER::IMP::AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc)
	{
		if (mapName2ID.find(cmdName) != mapName2ID.end())
		{
			throw std::runtime_error(std::string("failed to add gait, because \"")+cmdName+"\" already exists");
		}
		else
		{
			allGaits.push_back(gaitFunc);
			allParsers.push_back(parseFunc);

			mapName2ID.insert(std::make_pair(cmdName, allGaits.size() - 1));

			std::cout << cmdName << ":" << mapName2ID.at(cmdName) << std::endl;
		}
	};
	void ROBOT_SERVER::IMP::Start()
	{
		/*start sensors*/
		if (pImu)
		{
			pImu->Start();
		}

		server.SetOnReceivedConnection([](Aris::Core::CONN *pConn, const char *pRemoteIP, int remotePort)
		{
			Aris::Core::log(std::string("received connection, the ip is: ") + pRemoteIP);
			return 0;
		});
		server.SetOnReceiveRequest([this](Aris::Core::CONN *pConn, Aris::Core::MSG &msg)
		{
			Aris::Core::MSG ret;
			this->OnReceiveMsg(msg,ret);

			return ret;
		});
		server.SetOnLoseConnection([this](Aris::Core::CONN *pConn)
		{
			Aris::Core::log("lost connection");
			while (true)
			{
				try
				{
					pConn->StartServer(this->port.c_str());
					break;
				}
				catch (Aris::Core::CONN::START_SERVER_ERROR &e)
				{
					std::cout << e.what() << std::endl << "will try to restart in 1s" << std::endl;
					Aris::Core::Sleep(1000);
				}
			}

			return 0;
		});

		while (true)
		{
			try
			{
				server.StartServer(port.c_str());
				break;
			}
			catch (Aris::Core::CONN::START_SERVER_ERROR &e)
			{
				std::cout << e.what() << std::endl << "will restart in 5s" << std::endl;
#ifdef PLATFORM_IS_LINUX
				usleep(5000000);
#endif
			}
		}

#ifdef PLATFORM_IS_LINUX
		pController->Start();	
#endif
	}

	void ROBOT_SERVER::IMP::OnReceiveMsg(const Aris::Core::MSG &msg, Aris::Core::MSG &retError)
	{
		Aris::Core::MSG cmdMsg;
		try
		{
			std::string cmd;
			std::map<std::string, std::string> params;

			DecodeMsg(msg, cmd, params);
			GenerateCmdMsg(cmd, params, cmdMsg);
		}
		catch (std::exception &e)
		{
			cmdMsg.SetLength(0);
			retError.Copy(e.what());
			return;
		}

		cmdMsg.SetMsgID(0);

#ifdef PLATFORM_IS_LINUX
		this->pController->MsgPipe().SendToRT(cmdMsg);
#endif
	}
	void ROBOT_SERVER::IMP::DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string, std::string> &params)
	{
		std::vector<std::string> paramVector;
		int paramNum{0};

		/*将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5”*/
		if (msg.GetDataAddress()[msg.GetLength() - 1] == '\0')
		{
			std::string input{ msg.GetDataAddress() };
			std::stringstream inputStream{ input };
			std::string word;

			if (!(inputStream >> cmd))
			{
				throw std::logic_error(Aris::Core::log("invalid message from client, please at least contain a word"));
			};
			Aris::Core::log(std::string("received command string:")+msg.GetDataAddress());

			while (inputStream >> word)
			{
				paramVector.push_back(word);
				++paramNum;
			}
		}
		else
		{
			throw std::logic_error(Aris::Core::log("invalid message from client, please be sure that the command message end with char \'\\0\'"));
		}

		if (mapCmd.find(cmd) != mapCmd.end())
		{
			mapCmd.at(cmd)->root->Reset();
		}
		else
		{
			throw std::logic_error(Aris::Core::log(std::string("invalid command name, server does not have command \"") + cmd + "\""));
		}

		for (int i = 0; i<paramNum; ++i)
		{
			std::string str{ paramVector[i] };
			std::string paramName, paramValue;
			if (str.find("=") == std::string::npos)
			{
				paramName = str;
				paramValue = "";
			}
			else
			{
				paramName.assign(str, 0, str.find("="));
				paramValue.assign(str, str.find("=") + 1, str.size() - str.find("="));
			}

			if (paramName.size() == 0)
				throw std::logic_error("invalid param: what the hell, param should not start with '='");

			/*not start with '-'*/
			if (paramName.data()[0] != '-')
			{
				if (paramValue != "")
				{
					throw std::logic_error("invalid param: only param start with - or -- can be assigned a value");
				}

				for (auto c : paramName)
				{
					if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
					{
						params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
						mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
					}
					else
					{
						throw std::logic_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param" );
					}
				}

				continue;
			}

			/*all following part start with at least one '-'*/
			if (paramName.size() == 1)
			{
				throw std::logic_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
			}

			/*start with '-', but only one '-'*/
			if (paramName.data()[1] != '-')
			{
				if (paramName.size() != 2)
				{
					throw std::logic_error("invalid param: param start with single '-' must be an abbreviation");
				}

				char c = paramName.data()[1];

				if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
				{
					params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
					mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
				}
				else
				{
					throw std::logic_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
				}

				continue;
			}
			else
			{
				/*start with '--'*/
				if (paramName.size()<3)
				{
					throw std::logic_error("invalid param: symbol \"--\" must be followed by a full name of param");
				}

				std::string str = paramName;
				paramName.assign(str, 2, str.size() - 2);

				if (mapCmd.at(cmd)->allParams.find(paramName) != mapCmd.at(cmd)->allParams.end())
				{
					params.insert(make_pair(paramName, paramValue));
					mapCmd.at(cmd)->allParams.at(paramName)->Take();
				}
				else
				{
					throw std::logic_error(std::string("invalid param: param \"") + paramName + "\" is not a valid param");
				}

				

				continue;
			}
		}

		addAllDefault(mapCmd.at(cmd)->root.get(), params);

		std::cout << cmd << std::endl;

		int paramPrintLength;
		if (params.empty())
		{
			paramPrintLength = 2;
		}
		else
		{
			paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
			{
				return a.first.length() < b.first.length();
			})->first.length() + 2;
		}

		int maxParamNameLength{ 0 };
		for (auto &i : params)
		{
			std::cout << std::string(paramPrintLength-i.first.length(),' ') << i.first << " : " << i.second << std::endl;
		}
	}
	void ROBOT_SERVER::IMP::GenerateCmdMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::MSG &msg)
	{
		if (cmd == "en")
		{
			Robots::BASIC_FUNCTION_PARAM cmdParam;
			cmdParam.cmdType = ENABLE;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill_n(cmdParam.isMotorActive, 18, true);
				}
				else if (i.first == "first")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					std::fill_n(cmdParam.isMotorActive + 0, 3, true);
					std::fill_n(cmdParam.isMotorActive + 6, 3, true);
					std::fill_n(cmdParam.isMotorActive + 12, 3, true);
				}
				else if (i.first == "second")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					std::fill_n(cmdParam.isMotorActive + 3, 3, true);
					std::fill_n(cmdParam.isMotorActive + 9, 3, true);
					std::fill_n(cmdParam.isMotorActive + 15, 3, true);
				}
				else if (i.first == "motor")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					int id = { stoi(i.second) };
					cmdParam.isMotorActive[id] = true;
				}
			}

			msg.CopyStruct(cmdParam);
			return;
		}

		if (cmd == "ds")
		{
			Robots::BASIC_FUNCTION_PARAM cmdParam;
			cmdParam.cmdType = DISABLE;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill_n(cmdParam.isMotorActive, 18, true);
				}
				else if (i.first == "first")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					std::fill_n(cmdParam.isMotorActive + 0, 3, true);
					std::fill_n(cmdParam.isMotorActive + 6, 3, true);
					std::fill_n(cmdParam.isMotorActive + 12, 3, true);
				}
				else if (i.first == "second")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					std::fill_n(cmdParam.isMotorActive + 3, 3, true);
					std::fill_n(cmdParam.isMotorActive + 9, 3, true);
					std::fill_n(cmdParam.isMotorActive + 15, 3, true);
				}
				else if (i.first == "motor")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					int id = { stoi(i.second) };
					cmdParam.isMotorActive[id] = true;
				}
			}

			msg.CopyStruct(cmdParam);
			return;
		}

		if (cmd == "hm")
		{
			Robots::BASIC_FUNCTION_PARAM cmdParam;
			cmdParam.cmdType = HOME;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill_n(cmdParam.isMotorActive, 18, true);
				}
				else if (i.first == "first")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					std::fill_n(cmdParam.isMotorActive + 0, 3, true);
					std::fill_n(cmdParam.isMotorActive + 6, 3, true);
					std::fill_n(cmdParam.isMotorActive + 12, 3, true);
				}
				else if (i.first == "second")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					std::fill_n(cmdParam.isMotorActive + 3, 3, true);
					std::fill_n(cmdParam.isMotorActive + 9, 3, true);
					std::fill_n(cmdParam.isMotorActive + 15, 3, true);
				}
				else if (i.first == "motor")
				{
					std::fill_n(cmdParam.isMotorActive, 18, false);
					int id = { stoi(i.second) };
					cmdParam.isMotorActive[id] = true;
				}
			}

			msg.CopyStruct(cmdParam);
			return;
		}

		if (cmd == "rc")
		{
			Robots::RECOVER_PARAM cmdParam;
			cmdParam.cmdType = RECOVER;

			std::copy_n(this->recoverEE, 18, cmdParam.recoverPee);
			std::copy_n(this->alignIn, 18, cmdParam.alignPin);
			std::copy_n(this->alignEE, 18, cmdParam.alignPee);

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill_n(cmdParam.isLegActive, 6, true);
				}
				else if (i.first == "first")
				{
					cmdParam.isLegActive[0] = true;
					cmdParam.isLegActive[2] = true;
					cmdParam.isLegActive[4] = true;
				}
				else if (i.first == "second")
				{
					cmdParam.isLegActive[1] = true;
					cmdParam.isLegActive[3] = true;
					cmdParam.isLegActive[5] = true;
				}
				else if (i.first == "leg")
				{
					std::fill_n(cmdParam.isLegActive, 6, false);
					int id = { stoi(i.second) };
					cmdParam.isLegActive[id] = true;
				}
			}

			msg.CopyStruct(cmdParam);
			return;
		}

		auto cmdPair = this->mapName2ID.find(cmd);

		if (cmdPair != this->mapName2ID.end())
		{
			msg = this->allParsers.at(cmdPair->second).operator()(cmd, params);

			if (msg.GetLength() < sizeof(GAIT_PARAM_BASE))
			{
				throw std::logic_error(std::string("parse function of command \"") + cmdPair->first + "\" failed: because it returned invalid msg");
			}

			reinterpret_cast<GAIT_PARAM_BASE *>(msg.GetDataAddress())->cmdType=RUN_GAIT;
			reinterpret_cast<GAIT_PARAM_BASE *>(msg.GetDataAddress())->cmdID=cmdPair->second;
		}
		else
		{
			throw std::logic_error(std::string("command \"") + cmdPair->first + "\" does not have gait function, please AddGait() first");
		}
	}
	
	int ROBOT_SERVER::IMP::home(const Robots::BASIC_FUNCTION_PARAM *pParam, Aris::Control::CONTROLLER::DATA data)
	{
		bool isAllHomed = true;

		for (int i = 0; i < 18; ++i)
		{
			if (pParam->isMotorActive[i])
			{
				/*根据返回值来判断是否走到home了*/
				if ((pParam->count != 0) && (data.pMotionData->operator[](a2p(i)).ret == 0))
				{
					/*判断是否为第一次走到home,否则什么也不做，这样就会继续刷上次的值*/
					if (data.pMotionData->operator[](a2p(i)).cmd == Aris::Control::MOTION::HOME)
					{
						data.pMotionData->operator[](a2p(i)).cmd = Aris::Control::MOTION::RUN;
						data.pMotionData->operator[](a2p(i)).targetPos = data.pMotionData->operator[](a2p(i)).feedbackPos;
						data.pMotionData->operator[](a2p(i)).targetVel = 0;
						data.pMotionData->operator[](a2p(i)).targetCur = 0;
					}
				}
				else
				{
					isAllHomed = false;
					data.pMotionData->operator[](a2p(i)).cmd = Aris::Control::MOTION::HOME;

					if (pParam->count % 1000 == 0)
					{
						rt_printf("Unhomed motor, physical id: %d, absolute id: %d\n", a2p(i), i);
					}
				}
			}
		}

		return isAllHomed ? 0 : 1;
	};
	int ROBOT_SERVER::IMP::enable(const Robots::BASIC_FUNCTION_PARAM *pParam, Aris::Control::CONTROLLER::DATA data)
	{
		bool isAllEnabled = true;

		for (int i = 0; i < 18; ++i)
		{
			if (pParam->isMotorActive[i])
			{
				/*判断是否已经Enable了*/
				if ((pParam->count != 0) && (data.pMotionData->operator[](a2p(i)).ret == 0))
				{
					/*判断是否为第一次走到enable,否则什么也不做，这样就会继续刷上次的值*/
					if (data.pMotionData->operator[](a2p(i)).cmd == Aris::Control::MOTION::ENABLE)
					{
						data.pMotionData->operator[](a2p(i)).cmd = Aris::Control::MOTION::RUN;
						data.pMotionData->operator[](a2p(i)).mode = Aris::Control::MOTION::POSITION;
						data.pMotionData->operator[](a2p(i)).targetPos = data.pMotionData->operator[](a2p(i)).feedbackPos;
						data.pMotionData->operator[](a2p(i)).targetVel = 0;
						data.pMotionData->operator[](a2p(i)).targetCur = 0;
					}
				}
				else
				{
					isAllEnabled = false;
					data.pMotionData->operator[](a2p(i)).cmd = Aris::Control::MOTION::ENABLE;
					data.pMotionData->operator[](a2p(i)).mode = Aris::Control::MOTION::POSITION;

					if (pParam->count % 1000 == 0)
					{
						rt_printf("Unenabled motor, physical id: %d, absolute id: %d\n", a2p(i), i);
					}
				}
			}
		}

		return isAllEnabled ? 0 : 1;
	};
	int ROBOT_SERVER::IMP::disable(const Robots::BASIC_FUNCTION_PARAM *pParam, Aris::Control::CONTROLLER::DATA data)
	{
		bool isAllDisabled = true;

		for (int i = 0; i < 18; ++i)
		{
			if (pParam->isMotorActive[i])
			{
				/*判断是否已经Disabled了*/
				if ((pParam->count != 0) && (data.pMotionData->operator[](a2p(i)).ret == 0))
				{
					/*如果已经disable了，那么什么都不做*/
				}
				else
				{
					/*否则往下刷disable指令*/
					isAllDisabled = false;
					data.pMotionData->operator[](a2p(i)).cmd = Aris::Control::MOTION::DISABLE;

					if (pParam->count % 1000 == 0)
					{
						rt_printf("Undisabled motor, physical id: %d, absolute id: %d\n", a2p(i), i);
					}
				}
			}
		}

		return isAllDisabled ? 0 : 1;
	}
	int ROBOT_SERVER::IMP::recover(Robots::RECOVER_PARAM *pParam, Aris::Control::CONTROLLER::DATA data)
	{
		/*写入初值*/
		if (pParam->count == 0)
		{
			for (int i = 0; i<18; ++i)
			{
				pParam->beginPin[i] = data.pMotionData->operator[](i).feedbackPos/ meter2count;
				rt_printf("%f ", pParam->beginPin[i]);
			}
			rt_printf("\n");
		}
		
		
		const double pe[6]{ 0 };
		this->pServer->pRobot->SetBodyPe(pe);

		int leftCount = pParam->count < pParam->alignCount ? 0 : pParam->alignCount;
		int rightCount = pParam->count < pParam->alignCount ? pParam->alignCount : pParam->alignCount + pParam->recoverCount;

		double s = -(PI / 2)*cos(PI * (pParam->count - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

		for (int i = 0; i < 6; ++i)
		{
			if (pParam->isLegActive)
			{
				if (pParam->count < pParam->alignCount)
				{
					double pIn[3];
					for (int j = 0; j < 3; ++j)
					{
						pIn[j] = pParam->beginPin[i * 3 + j] * (cos(s) + 1) / 2 + pParam->alignPin[i * 3 + j] * (1 - cos(s)) / 2;
					}

					this->pServer->pRobot->pLegs[i]->SetPin(pIn);

				}
				else
				{
					double pEE[3];
					for (int j = 0; j < 3; ++j)
					{
						pEE[j] = pParam->alignPee[i * 3 + j] * (cos(s) + 1) / 2 + pParam->recoverPee[i * 3 + j] * (1 - cos(s)) / 2;
											
					}

					this->pServer->pRobot->pLegs[i]->SetPee(pEE);
				}
			}
		}

		//向下写入输入位置
		double pIn[18];
		pServer->pRobot->GetPin(pIn);
		
		for (int i = 0; i<18; ++i)
		{
			data.pMotionData->operator[](i).cmd = Aris::Control::MOTION::RUN;
			data.pMotionData->operator[](i).targetPos = static_cast<std::int32_t>(pIn[i] * meter2count);
		}

		return pParam->alignCount + pParam->recoverCount - pParam->count - 1;
	}
	int ROBOT_SERVER::IMP::runGait(Robots::GAIT_PARAM_BASE *pParam, Aris::Control::CONTROLLER::DATA data)
	{
		static double pBody[6]{ 0 }, vBody[6]{ 0 }, pEE[18]{ 0 }, vEE[18]{ 0 };
		if (pParam->count == 0)
		{
			pServer->pRobot->GetBodyPe(pBody);
			pServer->pRobot->GetPee(pEE);
			pServer->pRobot->GetBodyVel(vBody);
			pServer->pRobot->GetVee(vEE);

			rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
				, pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
				, pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
			rt_printf("%f %f %f %f %f %f\n"
				, pBody[0], pBody[1], pBody[2], pBody[3], pBody[4], pBody[5]);
		}

		std::copy_n(pEE, 18, pParam->beginPee);
		std::copy_n(vEE, 18, pParam->beginVee);
		std::copy_n(pBody, 6, pParam->beginBodyPE);
		std::copy_n(vBody, 6, pParam->beginBodyVel);

		//执行gait函数
		int ret = this->allGaits.at(pParam->cmdID).operator()(pServer->pRobot.get(),pParam);
		
		double pIn[18];
		pServer->pRobot->GetPin(pIn);

		//向下写入输入位置
		for (int i = 0; i<18; ++i)
		{
			data.pMotionData->operator[](i).cmd = Aris::Control::MOTION::RUN;
			data.pMotionData->operator[](i).targetPos = static_cast<std::int32_t>(pIn[i] * meter2count);
		}

		return ret;
	}
	
	int ROBOT_SERVER::IMP::execute_cmd(int count, char *cmd, Aris::Control::CONTROLLER::DATA data)
	{
		int ret;
		Robots::ALL_PARAM_BASE *pParam = reinterpret_cast<Robots::ALL_PARAM_BASE *>(cmd);
		pParam->count = count;

		switch (pParam->cmdType)
		{
		case ENABLE:
			ret = enable(static_cast<Robots::BASIC_FUNCTION_PARAM *>(pParam), data);
			break;
		case DISABLE:
			ret = disable(static_cast<Robots::BASIC_FUNCTION_PARAM *>(pParam), data);
			break;
		case HOME:
			ret = home(static_cast<Robots::BASIC_FUNCTION_PARAM *>(pParam), data);
			break;
		case RECOVER:
			ret = recover(static_cast<Robots::RECOVER_PARAM *>(pParam), data);
			break;
		case RUN_GAIT:
			ret = runGait(static_cast<Robots::GAIT_PARAM_BASE *>(pParam), data);
			break;
		default:
			rt_printf("unknown cmd type\n");
			ret = 0;
			break;
		}

		return ret;
	}
	int ROBOT_SERVER::IMP::tg(Aris::Control::CONTROLLER::DATA &data)
	{
		enum { CMD_POOL_SIZE = 50 };
		
		static const int cmdSize{ 8192 };
		static char cmdQueue[CMD_POOL_SIZE][cmdSize];

		static int currentCmd{ 0 };
		static int cmdNum{ 0 };
		static int count{ 0 };

		static int dspNum = 0;
		if (++dspNum % 1000 == 0)
		{
			rt_printf("pos is:%d \n",data.pMotionData->at(0).feedbackPos);
		}

		/*检查是否出错*/
		bool isAllNormal = true;
		for (auto &motData : *data.pMotionData)
		{
			if (motData.ret < 0)
			{
				isAllNormal = false;
				break;
			}
		}
		static int faultCount = 0;
		if (!isAllNormal)
		{
			if (faultCount++ % 100 == 0)
			{
				rt_printf("Some motor is in fault, now try to disable all motors\n");
				rt_printf("All commands in command queue are discarded\n");
			}
			for (auto &motData : *data.pMotionData)
			{
				motData.cmd = Aris::Control::MOTION::DISABLE;
			}
			
			cmdNum = 0;
		}
		else
		{
			faultCount = 0;
		}





		//查看是否有新cmd
		if (data.pMsgRecv)
		{
			if (cmdNum >= CMD_POOL_SIZE)
			{
				rt_printf("cmd pool is full, thus ignore last one\n");
			}
			else
			{
				data.pMsgRecv->Paste(cmdQueue[(currentCmd + cmdNum) % CMD_POOL_SIZE]);
				++cmdNum;
			}
		}

		//执行cmd queue中的cmd
		if (cmdNum>0)
		{
			if (Robots::ROBOT_SERVER::GetInstance()->pImp->execute_cmd(count, cmdQueue[currentCmd], data) == 0)
			{
				count = 0;
				currentCmd = (currentCmd + 1) % CMD_POOL_SIZE;
				cmdNum--;
				rt_printf("cmd finished\n");
			}
			else
			{
				count++;
			}

			if (count % 1000 == 0)
			{
				rt_printf("execute cmd in count: %d\n", count);
			}
		}

		//检查连续
		for (int i = 0; i<18; ++i)
		{
			if ((data.pLastMotionData->at(i).cmd == Aris::Control::MOTION::RUN)
				&& (data.pMotionData->at(i).cmd == Aris::Control::MOTION::RUN)
				&& (std::abs(data.pLastMotionData->at(i).targetPos - data.pMotionData->at(i).targetPos)>20000))
			{
				rt_printf("Data not continuous in count:%d\n", count);

				auto pR = ROBOT_SERVER::GetInstance()->pRobot.get();
				double pEE[18];
				double pBody[6];
				pR->GetPee(pEE);
				pR->GetBodyPe(pBody);

				rt_printf("The coming pee and body pe are:\n");
				rt_printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
					, pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
					, pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
				rt_printf("%f %f %f %f %f %f\n"
					, pBody[0], pBody[1], pBody[2], pBody[3], pBody[4], pBody[5]);

				rt_printf("The input of last and this count are:\n");
				for (int i = 0; i<18; ++i)
				{
					rt_printf("%d   %d\n", data.pLastMotionData->at(i).targetPos, data.pMotionData->at(i).targetPos);
				}

				rt_printf("All commands in command queue are discarded\n");
				cmdNum = 0;

				/*发现不连续，那么使用上一个成功的cmd，以便等待修复*/
				for (int i = 0; i < 18; ++i)
				{
					data.pMotionData->operator[](i) = data.pLastMotionData->operator[](i);
				}


				return 0;
			}
		}

		return 0;
	}

	ROBOT_SERVER * ROBOT_SERVER::GetInstance()
	{
		static ROBOT_SERVER instance;
		return &instance;
	}
	ROBOT_SERVER::ROBOT_SERVER():pImp(new IMP(this)){}
	ROBOT_SERVER::~ROBOT_SERVER(){}
	void ROBOT_SERVER::LoadXml(const char *fileName)
	{
		Aris::Core::DOCUMENT doc;

		if (doc.LoadFile(fileName) != 0)
		{
			throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
		}
		
		pImp->LoadXml(doc);
	}
	void ROBOT_SERVER::LoadXml(const Aris::Core::DOCUMENT &xmlDoc)
	{
		pImp->LoadXml(xmlDoc);
	}
	void ROBOT_SERVER::AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc)
	{
		pImp->AddGait(cmdName, gaitFunc, parseFunc);
	}
	void ROBOT_SERVER::Start()
	{
		pImp->Start();
	}
}








