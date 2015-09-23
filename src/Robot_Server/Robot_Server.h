#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <Platform.h>

#ifdef PLATFORM_IS_LINUX
#include <Aris_Control.h>
#endif

#include <Aris_ControlData.h>
#include <Aris_Socket.h>
#include <Robot_Gait.h>
#include <HexapodIII.h>
#include <string>
#include <sstream>
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
		static ROBOT_SERVER * GetInstance();

		template<typename T>
		void CreateRobot()
		{
			if (pRobot.get() == nullptr)
			{
				pRobot = std::unique_ptr<Robots::ROBOT_BASE>{ new T };
			}
			else
			{
				throw std::logic_error("already has a robot instance");
			}
		};
		void LoadXml(const char *fileName);
		void AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc);
		void Start();

	private:
		ROBOT_SERVER();
		~ROBOT_SERVER();

		ROBOT_SERVER(const ROBOT_SERVER &) = delete;
		ROBOT_SERVER &operator=(const ROBOT_SERVER &) = delete;

		std::unique_ptr<Robots::ROBOT_BASE> pRobot;

	private:
		class ROBOT_SERVER_IMP;
		ROBOT_SERVER_IMP *pImp;
	};

	

}

#endif

