#pragma once 

#define	_USE_MATH_DEFINES
#include <AngleAndComsumerPlugin.h>
#include <DataSources.hpp>
#include <string>
#include <set>
#include <mutex>
#include <thread>

#if defined(_WIN32)
class __declspec(dllexport) AACFromFilePlugin : public AngleAndComsumerPlugin
#else
class AACFromFilePlugin : public AngleAndComsumerPlugin
#endif
{
	// Private Variables:
	std::vector<std::string> _muscleNames;
	std::vector<std::string> _dofNames;
	std::map<std::string, double> _jointAngle, _jointTorqueFromCEINMS, _jointTorqueFromExternalOrID, _jointStiffness, _muscleActivation, _muscleForce, _normMuscleFiberLength, _normMuscleFiberVelocity, _position, _velocity, _acceleration, _grf;
	double _dataTimeStamp;
	double _outputTimeStamp;
	std::set<std::string> _emgChannels;
	std::mutex _dataSourcesAccess;
	std::shared_ptr<DataSources> _dataSources;
	std::map<std::string, std::map<std::string, double>> _subjectMuscleParameters;
	double _timeNow;
	bool _firstIteration;

public:
	/**
	* Constructor
	*/
	AACFromFilePlugin();

	/**
	* Destructor
	*/
	virtual ~AACFromFilePlugin();

	/**
	* Initialization method
	* @param xmlName Subject specific XML
	* @param executionName execution XML for CEINMS-RT software configuration
	*/
	void init(std::string& executionName);

	void reset()
	{
	}

	const std::map<std::string, double>& GetDataMap();

	/**
	* Get a set of the channel name
	*/
	const std::set<std::string>& GetNameSet();

	/**
	* Get the time stamp of the EMG capture.
	*/
	const double& getTime();
	// {
	// 	this->_dataTimeStamp = double(OSUtils::getTime());
	// 	return this->_dataTimeStamp;
	// }

	void stop();

	void setDirectory(std::string outDirectory, std::string inDirectory = std::string())
	{
	}

	void setVerbose(int verbose)
	{
	}

	void setRecord(bool record)
	{
	}

	const std::map<std::string, double>& GetDataMapTorque();

	void setMuscleName(const std::vector<std::string>& muscleNames)
	{
		this->_muscleNames = muscleNames;
	}

	void setDofName(const std::vector<std::string>& dofName)
	{
		this->_dofNames = dofName;
	}

	void setDofTorque(const std::vector<double>& dofTorque){
		for(int idx = 0; idx < dofTorque.size(); idx++ ){
			std::string& tag = this->_dofNames[idx];
			this->_jointTorqueFromCEINMS[tag] = dofTorque[idx];
		}
	}

	void setDofStiffness(const std::vector<double>& dofStiffness) {
		for(int idx = 0; idx < dofStiffness.size(); idx++ ){
			std::string& tag = this->_dofNames[idx];
			this->_jointStiffness[tag] = dofStiffness[idx];
		}
	}

	void setOutputTimeStamp(const double& timeStamp) {
		this->_outputTimeStamp = timeStamp;
	}

	void setMuscleForce(const std::vector<double>& muscleForce)  {
		for(int idx = 0; idx < muscleForce.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_muscleForce[tag] = muscleForce[idx];
		}
	}

	// NORMALIZED!!!! BE CAREFUL
	void setMuscleFiberLength(const std::vector<double>& muscleFiberLength) {
		for(int idx = 0; idx < muscleFiberLength.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_normMuscleFiberLength[tag] = muscleFiberLength[idx];
		}
	}

	void setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity)  {
		for(int idx = 0; idx < muscleFiberVelocity.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_normMuscleFiberVelocity[tag] = muscleFiberVelocity[idx];
		}
	}

	const std::vector<std::string>& GetDofName() { 
		return this->_dofNames; 
		}


	// Interface to provide an easy access method to collect data supplied by CEINMS
	double getDofAngle(const std::string& dofName) const; //{ return this->_jointAngle.at(dofName);}
	double getDofTorque(const std::string& dofName) const;// { return this->_jointTorqueFromExternalOrID.at(dofName);} // From ID or sensor
	double getCEINMSDofTorque(const std::string& dofName) const;// { return this->_jointTorqueFromExternalOrID.at(dofName);} // From ID or sensor
	double getDofStiffness(const std::string& dofName) const;// { return this->_jointStiffness.at(dofName);}
	double getMuscleForce(const std::string& muscleName) const;// { return this->_muscleForce.at(muscleName);}
	double getNormMuscleFiberLength(const std::string& muscleName) const;// { return this->_muscleFiberLength.at(muscleName);}
	double getNormMuscleFiberVelocity(const std::string& muscleName) const;// { return this->_muscleFiberVelocity.at(muscleName);}
	double getMuscleActivation(const std::string& muscleName) const;// { return this->_muscleActivation.at(muscleName);}
	double getChannelVelocity(const std::string& channelName) const;// { return this->_muscleFiberLength.at(muscleName);}
	double getChannelAcceleration(const std::string& channelName) const;// { return this->_muscleFiberVelocity.at(muscleName);}
	double getChannelPosition(const std::string& channelName) const;// { return this->_muscleActivation.at(muscleName);}
	// Optimization Cost Functions
	void stepDataSources(void);

	void setMuscleForcePassive(const std::vector<double>& muscleForcePassive);
	void setMuscleForceActive(const std::vector<double>& muscleForceActive);
	void setTendonStrain(const std::vector<double>& tendonStrain);
	const double& GetAngleTimeStamp();


};

