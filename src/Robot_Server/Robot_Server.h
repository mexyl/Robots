#ifndef ROBOT_SERVER_H
#define ROBOT_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>

#include <aris_imu.h>
#include <aris_message.h>
#include <Robot_Gait.h>

namespace Robots
{
	typedef std::function<Aris::Core::Msg(const std::string &cmd, const std::map<std::string, std::string> &params)> ParseFunc;

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
		void AddGait(std::string cmdName, GaitFunc gaitFunc, ParseFunc parseFunc);
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

