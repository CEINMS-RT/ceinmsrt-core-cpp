#include "EACFromFilePlugin.hpp"
#include <DataFromFileParser.hpp>
#include <boost/foreach.hpp>
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>
#include <map>

EACFromFilePlugin::EACFromFilePlugin() : _firstIteration{true}
{
	this->_outputTimeStamp = -1;
}

EACFromFilePlugin::~EACFromFilePlugin()
{	
}

void EACFromFilePlugin::init(std::string& subjectFilename, std::string& executionFilename)
{
	FileXMLParser parser(executionFilename);
	{
	std::lock_guard<std::mutex> lock(this->_dataSourcesAccess);
    parser.parseDataSources(this->_dataSources);
	}
	this->stepDataSources(); // No data is available before an initial step is given.
}


const std::map<std::string, double>& EACFromFilePlugin::GetDataMapEMG()
{
	return this->_muscleActivation;
}

const std::map<std::string, double>& EACFromFilePlugin::GetDataMapAngle()
{
	// Angle is the first thing collected, so time stepping is done here
	if(this->_firstIteration){
		this->_firstIteration = false;	// Dont step in the first time step. It was already done in the init function
	}else if((this->_outputTimeStamp != -1) && (this->_timeNow == this->_outputTimeStamp)){ // LmtMa thread may call this function several times.
														// To avoid that, it is cheched whether the ouput has already been
														// updated
		this->stepDataSources(); // Not thread safe at all. If more than one thread calls it, it will break.
	}

	std::lock_guard<std::mutex> lock(this->_dataSourcesAccess); // Data is copied from data sources
	return this->_jointAngle;
}

const std::map<std::string, double>& EACFromFilePlugin::GetDataMapTorque()
{
	std::lock_guard<std::mutex> lock(this->_dataSourcesAccess);
	return this->_jointTorqueFromExternalOrID;
}

void EACFromFilePlugin::stop()
{
}

const double& EACFromFilePlugin::getTime(){
	std::lock_guard<std::mutex> lock(this->_dataSourcesAccess);
	return this->_dataSources->currTimestamp();
}


double EACFromFilePlugin::getDofAngle(const std::string& dofName) const {
	if(this->_jointAngle.find(dofName) == this->_jointAngle.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find angle data for source " << dofName << " at timestep " << this->_dataSources->currTimestamp() << std::endl;
		return 0;
	}
	return this->_jointAngle.at(dofName);
}
double EACFromFilePlugin::getDofTorque(const std::string& dofName) const {
	if(this->_jointTorqueFromExternalOrID.find(dofName + std::string("_moment")) == this->_jointTorqueFromExternalOrID.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find torque data for source " << dofName << " at timestep " << this->_dataSources->currTimestamp()  << std::endl;
		return 0;
	}
	std::string tagName = dofName + "_moment";
	return this->_jointTorqueFromExternalOrID.at(tagName);
} // From ID or sensor
double EACFromFilePlugin::getCEINMSDofTorque(const std::string& dofName) const { 
	if(this->_jointTorqueFromCEINMS.find(dofName) == this->_jointTorqueFromCEINMS.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find torque data for source " << dofName << " at timestep " << this->_dataSources->currTimestamp()  << std::endl;
		return 0;
	}
	return this->_jointTorqueFromCEINMS.at(dofName);
} // From CEINMS core output
double EACFromFilePlugin::getDofStiffness(const std::string& dofName) const { 
	if(this->_jointStiffness.find(dofName) == this->_jointStiffness.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find stiffness data for source " << dofName << " at timestep " << this->_dataSources->currTimestamp()  << std::endl;
		return 0;
	}
	return this->_jointStiffness.at(dofName);
}
double EACFromFilePlugin::getMuscleForce(const std::string& muscleName) const { 
	if(this->_muscleForce.find(muscleName) == this->_muscleForce.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find force data for source " << muscleName << " at timestep " << this->_dataSources->currTimestamp()  << std::endl;
		return 0;
	}
	return this->_muscleForce.at(muscleName);
}
double EACFromFilePlugin::getNormMuscleFiberLength(const std::string& muscleName) const { 
	if(this->_normMuscleFiberLength.find(muscleName) == this->_normMuscleFiberLength.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find fiber length data for source " << muscleName << " at timestep " << this->_dataSources->currTimestamp()  << std::endl;
		return 0;
	}
	return this->_normMuscleFiberLength.at(muscleName) /* this->_subjectMuscleParameters.at(muscleName).at("optimalFiberLength")*/;
}
double EACFromFilePlugin::getNormMuscleFiberVelocity(const std::string& muscleName) const { 
	if(this->_normMuscleFiberVelocity.find(muscleName) == this->_normMuscleFiberVelocity.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find fiber velocity data for source " << muscleName  << " at timestep " << this->_dataSources->currTimestamp() << std::endl;
		return 0;
	}
	return this->_normMuscleFiberVelocity.at(muscleName);
}
double EACFromFilePlugin::getMuscleActivation(const std::string& muscleName) const { 
	if(this->_muscleActivation.find(muscleName) == this->_muscleActivation.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find activation data for source " << muscleName  << " at timestep " << this->_dataSources->currTimestamp() << std::endl;
		return 0;
	}
	return this->_muscleActivation.at(muscleName);
}

double EACFromFilePlugin::getChannelVelocity(const std::string& channelName) const{
	if(this->_velocity.find(channelName) == this->_velocity.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find velocity data for source " << channelName << " at timestep " << this->_dataSources->currTimestamp() << std::endl;
		return 0;
	}
	return this->_velocity.at(channelName);
}

double EACFromFilePlugin::getChannelAcceleration(const std::string& channelName) const{
	if(this->_acceleration.find(channelName) == this->_acceleration.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find acceleration data for source " << channelName << " at timestep " << this->_dataSources->currTimestamp() << std::endl;
		return 0;
	}
	return this->_acceleration.at(channelName);
}

double EACFromFilePlugin::getChannelPosition(const std::string& channelName) const{
	if(this->_position.find(channelName) == this->_position.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find position data for source " << channelName << " at timestep " << this->_dataSources->currTimestamp() << std::endl;
		return 0;
	}
	return this->_position.at(channelName);
}

void EACFromFilePlugin::stepDataSources(void){
	// Do I really want to have a copy of everything?
	std::lock_guard<std::mutex> lock(this->_dataSourcesAccess);
	this->_dataSources->step();
	this->_jointAngle = this->_dataSources->getAngle();
	this->_muscleActivation = this->_dataSources->getEMG();
	this->_jointTorqueFromExternalOrID = this->_dataSources->getTorque();
	this->_position = this->_dataSources->getPosition();
	this->_velocity = this->_dataSources->getVelocity();
	this->_acceleration = this->_dataSources->getAcceleration();
	this->_grf = this->_dataSources->getGRF();
	this->_timeNow = this->_dataSources->currTimestamp();
}


#ifdef UNIX
extern "C" EmgAndAngleAndComsumerPlugin * create() {
	return new EACFromFilePlugin;
}

extern "C" void destroy(EmgAndAngleAndComsumerPlugin * p) {
	delete p;
}
#endif

#if defined(WIN32) // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) EmgAndAngleAndComsumerPlugin * __cdecl create() {
	return new EACFromFilePlugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(EmgAndAngleAndComsumerPlugin * p) {
	delete p;
}
#endif


