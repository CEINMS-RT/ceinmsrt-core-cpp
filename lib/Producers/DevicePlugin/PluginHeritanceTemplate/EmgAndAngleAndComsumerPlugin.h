#ifndef EMGANDANGLEANDCOMSUMERPLUGIN_H_
#define EMGANDANGLEANDCOMSUMERPLUGIN_H_

#include <string>
#include <vector>
#include <map>
#include <string>

class EmgAndAngleAndComsumerPlugin
{
public:
	EmgAndAngleAndComsumerPlugin() {};
	~EmgAndAngleAndComsumerPlugin() {};
	virtual void init(std::string& xmlName, std::string& executionName) = 0;
	virtual void setDofName(const std::vector<std::string>& dofName) = 0;
	virtual void setDofTorque(const std::vector<double>& dofTorque) = 0;
	virtual void setDofStiffness(const std::vector<double>& dofStiffness) = 0;
	virtual void setOutputTimeStamp(const double& timeStamp) = 0;
	virtual void setMuscleName(const std::vector<std::string>& muscleName) = 0;
	virtual void setMuscleForce(const std::vector<double>& muscleForce) = 0;
	virtual void setMuscleFiberLength(const std::vector<double>& muscleFiberLength) = 0;
	virtual void setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity) = 0;
	//virtual const double& GetAngleTimeStamp() = 0; 
	virtual const double& getTime() = 0;
	virtual const std::vector<std::string>& GetDofName() = 0;
	virtual const std::map<std::string, double>& GetDataMapEMG() = 0; //< For having Data and name correspondence
	virtual const std::map<std::string, double>& GetDataMapAngle() = 0;
	virtual const std::map<std::string, double>& GetDataMapTorque() = 0;
	virtual void stop() = 0;
	virtual void setDirectory(std::string outDirectory, std::string inDirectory = std::string()) = 0;
	virtual void setVerbose(int verbose) = 0;
	virtual void setRecord(bool record) = 0;
};

typedef EmgAndAngleAndComsumerPlugin* create_e();
typedef void destroy_e(EmgAndAngleAndComsumerPlugin*);

#endif /* EMGANDANGLEANDCOMSUMERPLUGIN_H_ */
