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

#ifndef PRODUCERSPLUGINVIRTUAL_H_
#define PRODUCERSPLUGINVIRTUAL_H_

#include <string>
#include <map>
#include <vector>
#include <set>

/**
 *@TODO Add joint Torque
 **/

using namespace std;

class ProducersPluginVirtual
{
	public:
		ProducersPluginVirtual() {};
		~ProducersPluginVirtual() {};
		virtual void init ( std::string executionXMLFile = std::string(), std::string subjectCEINMSXMLFile = std::string() ) = 0;
		virtual void reset() = 0;
		virtual void stop() = 0;
		virtual const map<string, double>& GetDataMap() = 0; //< For having Data and name correspondence
		virtual const map<string, double>& GetDataMapTorque() = 0; //< For having Data and name correspondence
		virtual const double& getTime() = 0;
		virtual void setDirectories ( std::string outDirectory, std::string inDirectory = std::string() ) = 0;
		virtual void setVerbose ( int verbose ) = 0;
		virtual void setRecord ( bool record ) = 0;
};

typedef ProducersPluginVirtual* create_t();
typedef void destroy_t ( ProducersPluginVirtual* );

#endif /* PRODUCERSPLUGINVIRTUAL_H_ */
