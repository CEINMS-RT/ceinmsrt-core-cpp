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

#ifndef ANGLE0PLUGIN_H_
#define ANGLE0PLUGIN_H_

#include "ProducersPluginVirtual.h"
#include <time.h>
#ifdef UNIX
	#include <sys/time.h>
#endif
#include <stdio.h>
#include <iostream> 
#include "NMSmodel.hxx"
#ifdef WIN32
	#include <windows.h>
#endif
#include <ctime>
#include <iomanip>

#ifdef WIN32
class __declspec(dllexport) Angle0plugin : public ProducersPluginVirtual {
#endif
#ifdef UNIX
	class  Angle0plugin : public ProducersPluginVirtual {
#endif
public:
	Angle0plugin();
	virtual ~Angle0plugin();

	void init(string xmlName, string executionName);

	const map<string, double>& GetDataMap() //< For having Data and name correspondence
	{
		return mapData_;
	}

	const set<string>& GetNameSet()
	{
		return nameSet_;
	}

	const double& getTime();
	
	void reset()
	{
		
	}
	
	
	
	void stop()
	{
	  
	}
	
	void setDirectories ( std::string outDirectory, std::string inDirectory = std::string() )
	{
	  
	}

	void setVerbose ( int verbose )
	{
	  
	}

	void setRecord ( bool record )
	{
	  
	}
	
	const map<string, double>& GetDataMapTorque()
	{
		return _torque;
	}

protected:
	map<string, double> mapData_;
	set<string> nameSet_;
#ifdef UNIX
	timeval tInit_;
#endif
	double timeNow_;
	map<string, double> _torque;
};

#endif /* ANGLE0PLUGIN_H_ */
