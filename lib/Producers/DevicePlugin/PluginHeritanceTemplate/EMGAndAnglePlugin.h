#ifndef EMGANDANGLEPLUGIN_H_
#define EMGANDANGLEPLUGIN_H_

#include <string>
#include <map>
#include <vector>
#include <set>
#include <string>

class EMGAndAnglePlugin
{
public:
	EMGAndAnglePlugin() {};
	~EMGAndAnglePlugin() {};
	virtual void init(std::string executionXMLFile = std::string(), std::string subjectCEINMSXMLFile = std::string()) = 0;
	virtual void reset() = 0;
	virtual void stop() = 0;
	virtual const std::map<std::string, double>& GetDataMap() = 0; //< For having Data and name correspondence
	virtual const std::map<std::string, double>& GetDataMapPos() = 0; //< For having Data and name correspondence
	virtual const double& getTime() = 0;
	virtual void setDirectories(std::string outDirectory, std::string inDirectory = std::string()) = 0;
	virtual void setVerbose(int verbose) = 0;
	virtual void setRecord(bool record) = 0;
};

#endif 