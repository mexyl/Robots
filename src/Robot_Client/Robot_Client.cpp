#include <Platform.h>

#include <iostream>
#include <list>
#include <vector>
#include <map>
#include <cstring>
#include <string>
#include <memory>
#include <algorithm>
#include <cstdint>

#include <Aris_Socket.h>
#include <Aris_XML.h>
#include <Aris_Core.h>

using namespace std;

class NODE
{
public:
	NODE* AddChildGroup(const char *Name);
	NODE* AddChildUnique(const char *Name);
	NODE* AddChildParam(const char *Name);
	NODE* FindChild(const char *Name)
	{
		auto result=find_if(children.begin(),children.end(),[Name](std::unique_ptr<NODE> &node)
		{
			bool is=!strcmp(node->name.c_str(),Name);

			return is;
		});

		if(result!=children.end())
		{
			return result->get();
		}
		else
		{
			return nullptr;
		}
	};

	bool IsTaken(){return isTaken;};
	void Take();

public:
	NODE(NODE*father,const char *Name):name(Name){this->father=father;};
	virtual ~NODE(){};

private:
	string name;
	NODE* father;
	std::vector<std::unique_ptr<NODE> > children;

	bool isTaken{false};

	friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string,NODE *> &allParams, map<char, string>& shortNames);
	friend void addAllDefault(NODE *pNode, map<string,string> &params);
};
class ROOT_NODE:public NODE
{
public:
	ROOT_NODE(const char *Name):NODE(nullptr,Name){};

private:
	NODE *pDefault;

	friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string,NODE *> &allParams, map<char, string>& shortNames);
	friend void addAllDefault(NODE *pNode, map<string,string> &params);
};
class GROUP_NODE:public NODE
{
public:
	GROUP_NODE(NODE*father,const char *Name):NODE(father,Name){};
};
class UNIQUE_NODE:public NODE
{
public:
	UNIQUE_NODE(NODE*father,const char *Name):NODE(father,Name){};

private:
	NODE *pDefault;

	friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string,NODE *> &allParams, map<char, string>& shortNames);
	friend void addAllDefault(NODE *pNode, map<string,string> &params);
};
class PARAM_NODE:public NODE
{
public:
	PARAM_NODE(NODE*father,const char *Name):NODE(father,Name){};
private:
	std::string type;
	std::string defaultValue;
	std::string minValue,maxValue;

	friend void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string,NODE *> &allParams, map<char, string>& shortNames);
	friend void addAllDefault(NODE *pNode, map<string,string> &params);
};

NODE* NODE::AddChildGroup(const char *Name)
{
	this->children.push_back(std::unique_ptr<NODE>(new GROUP_NODE(this,Name)));
	return children.back().get();
};
NODE* NODE::AddChildUnique(const char *Name)
{
	this->children.push_back(std::unique_ptr<NODE>(new UNIQUE_NODE(this,Name)));
	return children.back().get();
}
NODE* NODE::AddChildParam(const char *Name)
{
	this->children.push_back(std::unique_ptr<NODE>(new PARAM_NODE(this,Name)));
	return children.back().get();
}
void NODE::Take()
{
	if(dynamic_cast<ROOT_NODE*>(this))
	{
		if(this->isTaken)
		{
			throw std::logic_error("some param contradicted with each other in root node");
		}
		else
		{
			this->isTaken=true;
			return;
		}
	}
	else if(dynamic_cast<GROUP_NODE*>(this))
	{
		if(this->isTaken)
		{
			return;
		}
		else
		{
			this->isTaken=true;
			father->Take();
		}
	}
	else
	{
		if(this->isTaken)
		{
			throw std::logic_error("some param contradicted with each other in other node");
		}
		else
		{
			this->isTaken=true;
			father->Take();
		}
	}
}


void addAllParams(Aris::Core::ELEMENT *pEle, NODE *pNode, map<string,NODE *> &allParams, map<char, string>& shortNames)
{
	/*add all children*/
	for (auto pChild = pEle->FirstChildElement();
			pChild != nullptr;
			pChild = pChild->NextSiblingElement())
	{
		/*check if children already has this value*/
		if(pNode->FindChild(pChild->Name()))
		{
			throw logic_error("node already exist");
		}

		/*set all children*/
		if(pChild->Attribute("type","group"))
		{
			addAllParams(pChild, pNode->AddChildGroup(pChild->Name()), allParams, shortNames);
		}
		else if(pChild->Attribute("type","unique"))
		{
			addAllParams(pChild, pNode->AddChildUnique(pChild->Name()), allParams,  shortNames);
		}
		else
		{
			/*now the pChild is a param_node*/
			NODE * insertNode;

			if(allParams.find(string(pChild->Name()))!=allParams.end())
			{
				throw std::logic_error("repeated param");
			}
			else
			{
				insertNode=pNode->AddChildParam(pChild->Name());
				allParams.insert(pair<string,NODE *>(string(pChild->Name()),insertNode));
			}

			/*set abbreviation*/
			if(pChild->Attribute("abbreviation"))
			{
				if(shortNames.find(*pChild->Attribute("abbreviation"))!=shortNames.end())
				{
					throw std::logic_error("repeated abbreviations");
				}
				else
				{
					char abbr=*pChild->Attribute("abbreviation");
					shortNames.insert(pair<char,string>
					(
							abbr,string(pChild->Name())
					));
				}
			}





			/*set values*/
			if(pChild->Attribute("type"))
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->type=string(pChild->Attribute("type"));
			}
			else
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->type="";
			}

			if(pChild->Attribute("default"))
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->defaultValue=string(pChild->Attribute("default"));
			}
			else
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->defaultValue="";
			}

			if(pChild->Attribute("maxValue"))
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->maxValue=string(pChild->Attribute("maxValue"));
			}
			else
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->maxValue="";
			}

			if(pChild->Attribute("minValue"))
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->minValue=string(pChild->Attribute("minValue"));
			}
			else
			{
				dynamic_cast<PARAM_NODE*>(insertNode)->minValue="";
			}
		}
	}

	/*set all values*/
	if(dynamic_cast<ROOT_NODE*>(pNode))
	{
		if(pEle->Attribute("default"))
		{
			if(pNode->FindChild(pEle->Attribute("default")))
			{
				dynamic_cast<ROOT_NODE*>(pNode)->pDefault=pNode->FindChild(pEle->Attribute("default"));
			}
			else
			{
				throw logic_error("can't find default child node");
			}
		}
		else
		{
			if(pNode->children.empty())
			{
				dynamic_cast<ROOT_NODE*>(pNode)->pDefault=nullptr;
			}
			else
			{
				dynamic_cast<ROOT_NODE*>(pNode)->pDefault=pNode->children.begin()->get();
			}
		}
	}

	if(dynamic_cast<UNIQUE_NODE*>(pNode))
	{
		if(pEle->Attribute("default"))
		{
			if(pNode->FindChild(pEle->Attribute("default")))
			{
				dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault=pNode->FindChild(pEle->Attribute("default"));
			}
			else
			{
				throw logic_error("can't find default child node");
			}
		}
		else
		{
			if(pNode->children.empty())
			{
				throw logic_error("unique node must have more than 1 child");
			}
			else
			{
				dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault=pNode->children.begin()->get();
			}
		}
	}
}
void addAllDefault(NODE *pNode, map<string,string> &params)
{
	if(pNode->isTaken)
	{
		if(dynamic_cast<ROOT_NODE*>(pNode))
		{
			auto found=find_if(pNode->children.begin(),pNode->children.end(),[](unique_ptr<NODE> &a)
			{
				return a->isTaken;
			});

			addAllDefault(found->get(),params);
		}

		if(dynamic_cast<UNIQUE_NODE*>(pNode))
		{
			auto found=find_if(pNode->children.begin(),pNode->children.end(),[](unique_ptr<NODE> &a)
			{
				return a->isTaken;
			});

			addAllDefault(found->get(),params);
		}

		if(dynamic_cast<GROUP_NODE*>(pNode))
		{
			for(auto &i:pNode->children)
				addAllDefault(i.get(),params);
		}

		if(dynamic_cast<PARAM_NODE*>(pNode))
		{
			if(params.at(pNode->name)=="")
			{
				params.at(pNode->name)=dynamic_cast<PARAM_NODE*>(pNode)->defaultValue;
			}
			return;
		}
	}
	else
	{
		if(dynamic_cast<ROOT_NODE*>(pNode))
		{
			if(dynamic_cast<ROOT_NODE*>(pNode)->pDefault)
			{
				addAllDefault(dynamic_cast<ROOT_NODE*>(pNode)->pDefault,params);
			}

			pNode->isTaken=true;
		}

		if(dynamic_cast<UNIQUE_NODE*>(pNode))
		{
			addAllDefault(dynamic_cast<UNIQUE_NODE*>(pNode)->pDefault,params);

			pNode->isTaken=true;
		}

		if(dynamic_cast<GROUP_NODE*>(pNode))
		{
			for(auto &i:pNode->children)
				addAllDefault(i.get(),params);

			pNode->isTaken=true;
		}

		if(dynamic_cast<PARAM_NODE*>(pNode))
		{
			params.insert(make_pair(pNode->name,dynamic_cast<PARAM_NODE*>(pNode)->defaultValue));
			pNode->isTaken=true;
		}
	}



}

namespace Robots
{
	int SendRequest(int argc, char *argv[], const char *xmlFileName)
	{
		Aris::Core::DOCUMENT doc;

		if (doc.LoadFile(xmlFileName) != 0)
			throw std::logic_error("failed to read configuration xml file");

		/*get param tree of this cmd*/


		auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

		if (pCmds == nullptr)
			throw std::logic_error("invalid client.xml");

		auto thisCmd = pCmds->FirstChildElement(argv[0]);
		if (thisCmd == nullptr)
			throw std::logic_error("client.xml:not find this cmd");

		ROOT_NODE root(argv[0]);

		map<string, NODE *> allParams;
		map<char, string> shortNames;

		addAllParams(thisCmd, &root, allParams, shortNames);


		/*get all params of this cmd*/
		string cmd(thisCmd->Name());

		map<string, string> params;
		for (int i = 1; i<argc; ++i)
		{
			string str{ argv[i] };
			string paramName, paramValue;
			if (str.find("=") == string::npos)
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
				throw logic_error("invalid param:what the hell, param should not start with '='");

			/*not start with '-'*/
			if (paramName.data()[0] != '-')
			{
				if (paramValue != "")
				{
					throw logic_error("invalid param:only param start with - or -- can be assigned a value");
				}


				for (auto c : paramName)
				{
					params.insert(make_pair(shortNames.at(c), string("")));
					allParams.at(shortNames.at(c))->Take();
				}

				continue;
			}



			/*all following part start with at least one '-'*/
			if (paramName.size() == 1)
			{
				throw logic_error("invalid param:param must have a name");
			}

			/*start with '-', but only one '-'*/
			if (paramName.data()[1] != '-')
			{
				if (paramName.size() != 2)
				{
					throw std::logic_error("param start with single '-' must is an abbreviation");
				}

				char c = paramName.data()[1];

				params.insert(make_pair(shortNames.at(c), paramValue));
				allParams.at(shortNames.at(c))->Take();

				continue;
			}
			else
			{
				/*start with '--'*/
				if (paramName.size()<3)
				{
					throw std::logic_error("param start with single '--' must is a full name");
				}

				string str = paramName;
				paramName.assign(str, 2, str.size() - 2);

				params.insert(make_pair(paramName, paramValue));
				allParams.at(paramName)->Take();

				continue;
			}
		}





		/*add all default values*/
		addAllDefault(&root, params);


		/*pack all datas*/
		Aris::Core::MSG msg{ 0,0 };

		int32_t size = cmd.size() + 1;

		msg.CopyStructMore(size);
		msg.CopyMore(cmd.c_str(), size);


		size = params.size();
		msg.CopyStructMore(size);

		for (auto &i : params)
		{
			size = i.first.size() + 1;
			msg.CopyStructMore(size);
			msg.CopyMore(i.first.c_str(), i.first.size() + 1);
			size = i.second.size() + 1;
			msg.CopyStructMore(size);
			msg.CopyMore(i.second.c_str(), i.second.size() + 1);
		}

		for (auto &i : params)
		{
			cout << i.first << ":" << i.second << endl;
		}

		
		std::string ip = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection")->Attribute("IP");
		std::string port = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection")->Attribute("Port");

		Aris::Core::CONN conn;

		conn.Connect(ip.c_str(), port.c_str());
		conn.SendRequest(msg);

		cout << "send finished" << endl;

		return 0;
	}
}

