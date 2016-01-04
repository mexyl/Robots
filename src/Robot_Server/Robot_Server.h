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
	typedef std::function<Aris::Core::Msg(const std::string &cmd, const std::map<std::string, std::string> &params)> PARSE_FUNC;

	class RobotServer
	{
	public:
		static RobotServer * GetInstance();

		template<typename T>
		void CreateRobot()
		{
			if (pRobot)
				throw std::logic_error("already has a robot instance");
			else
				pRobot.reset(new T);
		};
		void LoadXml(const char *fileName);
		void LoadXml(const Aris::Core::XmlDocument &xmlDoc);
		void AddGait(std::string cmdName, GaitFunc gaitFunc, PARSE_FUNC parseFunc);
		void Start();
		void Stop();

	private:
		RobotServer();
		~RobotServer();
		RobotServer(const RobotServer &) = delete;
		RobotServer &operator=(const RobotServer &) = delete;

		std::unique_ptr<Robots::RobotBase> pRobot;

	private:
		class Imp;
		std::unique_ptr<Imp> pImp;
	};
}

#endif

