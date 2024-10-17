#ifndef PRODUCERSANDCONSUMERPLUGIN_H_
#define PRODUCERSANDCONSUMERPLUGIN_H_

#include <string>
#include <vector>
#include <map>
#include <string>

class ProducersAndConsumerPlugin
{
public:
	ProducersAndConsumerPlugin() {};
	~ProducersAndConsumerPlugin() {};
	virtual void init(int portno) = 0;
	virtual void setDofName(const std::vector<std::string>& dofName) = 0;
	virtual void setDofTorque(const std::vector<double>& dofTorque) = 0;
	virtual void setDofStiffness(const std::vector<double>& dofStiffness) = 0;
	virtual void setOutputTimeStamp(const double& timeStamp) = 0;
	virtual void setMuscleName(const std::vector<std::string>& muscleName) = 0;
	virtual const double& GetAngleTimeStamp() = 0;
	virtual const std::vector<std::string>& GetDofName() = 0;
	virtual const std::map<std::string, double>& GetDataMapAngle() = 0; //< For having Data and name correspondence
	virtual const std::map<std::string, double>& GetDataMapEMG() = 0;
	virtual const std::map<std::string, double>& GetDataMapTorque() = 0;
	virtual void stop() = 0;
	virtual void setDirectory(std::string outDirectory, std::string inDirectory = std::string()) = 0;
	virtual void setVerbose(int verbose) = 0;
	virtual void setRecord(bool record) = 0;
};

#endif