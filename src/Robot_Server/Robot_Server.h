#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>

#include <Aris_IMU.h>
#include <Aris_Message.h>
#include <Robot_Gait.h>

namespace Robots
{
	typedef std::function<Aris::Core::MSG(const std::string &cmd, const std::map<std::string, std::string> &params)> PARSE_FUNC;

	class ROBOT_SERVER
	{
	public:
		static ROBOT_SERVER * GetInstance();

		template<typename T>
		void CreateRobot()
		{
			if (pRobot.get())
			{
				throw std::logic_error("already has a robot instance");
			}
			else
			{
				pRobot = std::unique_ptr<Robots::ROBOT_BASE>{ new T };
			}
		};
		void LoadXml(const char *fileName);
		void LoadXml(const Aris::Core::DOCUMENT &xmlDoc);
		void AddGait(std::string cmdName, GAIT_FUNC gaitFunc, PARSE_FUNC parseFunc);
		void Start();
		void Stop();

	private:
		ROBOT_SERVER();
		~ROBOT_SERVER();
		ROBOT_SERVER(const ROBOT_SERVER &) = delete;
		ROBOT_SERVER &operator=(const ROBOT_SERVER &) = delete;

		std::unique_ptr<Robots::ROBOT_BASE> pRobot;

	private:
		class IMP;
		std::unique_ptr<IMP> pImp;
	};
}

#endif

