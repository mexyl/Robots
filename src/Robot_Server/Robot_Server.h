#include <Aris_ControlData.h>
#include <Aris_Socket.h>
#include <Robot_Gait.h>
#include <HexapodIII.h>
#include <string>
#include <map>

#include <memory>

namespace Robots
{
	typedef std::function<Aris::Core::MSG(const std::string &cmd, const std::map<std::string, std::string> &params)> PARSE_FUNC;
	
	enum ROBOT_CMD_ID
	{
		ENABLE,
		DISABLE,
		HOME,
		RESET_ORIGIN,
		RUN_GAIT,

		ROBOT_CMD_COUNT
	};
	
	class ROBOT_SERVER
	{
	public:
		static ROBOT_SERVER * GetInstance()
		{
			static ROBOT_SERVER instance;
			return &instance;
		}

		template <typename T>
		void CreateRobot(const char *xmlFile = nullptr) 
		{
			if (pRobot.get() == nullptr)
			{
				pRobot = std::unique_ptr<Robots::ROBOT_BASE>(new T);

				if (xmlFile)
				{
					pRobot->LoadXml(xmlFile);
				}
			}
		};
		void AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc)
		{
			if (mapName2ID.find(cmdName) != mapName2ID.end())
			{
				allGaits.push_back(gaitFunc);
				allParsers.push_back(parseFunc);

				mapName2ID.insert(std::make_pair(cmdName, allGaits.size() - 1));
			}
		};
		void ExecuteMsg(const Aris::Core::MSG &m);
		void Start();
		int execute_cmd(int count, char *cmd, Aris::RT_CONTROL::CMachineData &data);

	private:
		ROBOT_SERVER() = default;
		ROBOT_SERVER(const ROBOT_SERVER&) = delete;

		void DecodeMsg(const Aris::Core::MSG &msg, std::string &cmd, std::map<std::string, std::string> &params);
		void GenerateCmdMsg(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::MSG &msg);
		int runGait(Robots::ROBOT_BASE *pRobot, const Robots::GAIT_PARAM_BASE *pParam, Aris::RT_CONTROL::CMachineData &data);
		
	private:
		std::unique_ptr<Robots::ROBOT_BASE> pRobot;
		std::map<std::string, int> mapName2ID;
		std::vector<GAIT_FUNC> allGaits;
		std::vector<PARSE_FUNC> allParsers;

		Aris::Core::CONN server;
#ifdef PLATFORM_IS_LINUX
		Aris::RT_CONTROL::ACTUATION cs;
#endif
	};

}



