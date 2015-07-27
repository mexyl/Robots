#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <Aris_XML.h>


using namespace std;

void addParamGroup(Aris::Core::ELEMENT *group, int argc, char *argv[])
{




};

void makeCmd(Aris::Core::ELEMENT *cmd)
{
	std::ofstream file;

	file.open(std::string(cmd->Name())+string(".cpp"), std::ios::out | std::ios::trunc);

	file<<"#include <iostream>"<<endl;
	file<<"#include <list>"<<endl;
	file<<"#include <map>"<<endl;
	file<<"#include <cstring>"<<endl;
	file<<"#include <string>"<<endl<<endl;

	file<<"using namespace std;"<<endl<<endl;

	file<<"int main(int argc, char *argv[])"<<endl<<"{"<<endl;

	file<<"\t"<<"map<string,string> params;"<<endl;

	file<<"\t"<<"for(int i=1;i<argc;++i)"<<endl;
	file<<"\t"<<"{"<<endl;

	file<<"\t\t"<<"string str{argv[i]};"<<endl;

	file<<"\t\t"<<"string paramName,paramValue;"<<endl;

	file<<"\t\t"<<"if(str.find(\"=\")==string::npos)"<<endl;
	file<<"\t\t"<<"{"<<endl;

	file<<"\t\t\t"<<"paramName = str;"<<endl;
	file<<"\t\t\t"<<"paramValue = \"\";"<<endl;

	file<<"\t\t"<<"}"<<endl;
	file<<"\t\t"<<"else"<<endl;
	file<<"\t\t"<<"{"<<endl;

	file<<"\t\t\t"<<"paramName.assign(str,0,str.find(\"=\"));"<<endl;
	file<<"\t\t\t"<<"paramValue.assign(str,str.find(\"=\")+1,str.size()-str.find(\"=\"));"<<endl;

	file<<"\t\t"<<"}"<<endl;

	file<<"\t\t"<<"if(paramName.size()==0)"<<endl;
	file<<"\t\t\t"<<"throw logic_error(\"invalid param:param must have a name\");"<<endl;

	file<<"\t\t"<<"char shortName = 0;"<<endl;

	file<<"\t\t"<<"if(paramName.size()==1)"<<endl;
	file<<"\t\t"<<"{"<<endl;

	file<<"\t\t\t"<<"shortName=paramName.data()[0];"<<endl;

	file<<"\t\t"<<"}"<<endl;

	file<<"\t\t"<<"if(paramName.size()==2)"<<endl;
	file<<"\t\t"<<"{"<<endl;

	file<<"\t\t\t"<<"if(paramName.data()[0]!='-')"<<endl;
	file<<"\t\t\t\t"<<"throw logic_error(\"invalid param\");"<<endl;

	file<<"\t\t\t"<<"shortName=paramName.data()[1];"<<endl;

	file<<"\t\t"<<"}"<<endl;

	file<<"\t\t"<<"if(shortName)"<<endl;
	file<<"\t\t"<<"{"<<endl;

	file<<"\t\t\t"<<"paramName.clear();"<<endl;
	file<<"\t\t\t"<<"paramName.resize(1,shortName);"<<endl;

	file<<"\t\t"<<"} else"<<endl;
	file<<"\t\t"<<"{"<<endl;

	file<<"\t\t\t"<<"if((paramName.data()[0]!='-')||(paramName.data()[1]!='-'))"<<endl;
	file<<"\t\t\t\t"<<"throw logic_error(\"invalid param, must have -- prefix for full name\");"<<endl;

	file<<"\t\t\t"<<"string str=paramName;"<<endl;
	file<<"\t\t\t"<<"paramName.assign(str,2,str.size()-2);"<<endl;

	file<<"\t\t"<<"}"<<endl;

	file<<"\t\t"<<"cout<<paramName<<endl;"<<endl;
	file<<"\t\t"<<"cout<<paramValue<<endl;"<<endl;


	file<<"\t"<<"}"<<endl<<endl;

/*
	for(Aris::Core::ELEMENT *group = cmd->FirstChildElement();
				group != nullptr;
				group = group->NextSiblingElement())
	{
		addParamGroup(cmd);
	}
*/

	//file<<"\tAris::Core::MSG msg;"<<endl;

	file<<"\treturn 0;"<<endl;
	file<<"}"<<endl;

	file.close();
};




int main(int argc, char *argv[])
{
	if(argc<3)
		throw std::logic_error("too little args");

	Aris::Core::DOCUMENT doc;

	if(doc.LoadFile(argv[1])!=0)
		throw std::logic_error("xml not exist");

	Aris::Core::ELEMENT *pCmds = doc.FirstChildElement("Commands");

	for(Aris::Core::ELEMENT *cmd = pCmds->FirstChildElement();
		cmd != nullptr;
		cmd = cmd->NextSiblingElement())
	{
		makeCmd(cmd);
	}



	std::cout<<"hello word"<<std::endl;
	return 0;
}
