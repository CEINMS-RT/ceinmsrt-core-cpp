// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

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