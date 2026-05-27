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

#ifndef ANGLEFROMDEVICE_H_
#define ANGLEFROMDEVICE_H_

#include "CommonCEINMS.h"
#include "DynLib.h"
#include <vector>
#include <map>
#include <string>
#include "SyncTools.h"
#include "DynLib.h"
#include "AngleFromX.h"
#include "execution.hxx"
#include "executionIK_ID.hxx"
#include <csignal>
#include <stddef.h>
#include "AngleAndComsumerPlugin.h"
#include "EmgAndAngleAndComsumerPlugin.h"
#include "ExecutionXmlReader.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ProducersPluginVirtual.h>


class AngleFromDevice: public DynLib<ProducersPluginVirtual>, public AngleFromX
{
public:
	AngleFromDevice(){}
	AngleFromDevice(string xmlName, string executionName, bool record = false,
		std::string recordDirectory = std::string(), std::string processDirectory = std::string(), bool process = false);
	virtual ~AngleFromDevice();
	void operator()();
	
	void setPluginAAC(AngleAndComsumerPlugin* pluginAAC)
	{
		_pluginAAC = pluginAAC;
	}

	void setPluginEAC(EmgAndAngleAndComsumerPlugin* pluginEAC)
	{
		_pluginEAC = pluginEAC;
	}
	
private:
	std::vector<std::string> dofNames_;
	bool firstPass_;
	string xmlName_;
	string executionName_;
	string osimFile_;
	string labFile_;
	int _verbose;
	bool _gui;
	bool _process;
	std::string _processDirectory;
	bool _record;
	std::string _recordDirectory;
	AngleAndComsumerPlugin* _pluginAAC;
	EmgAndAngleAndComsumerPlugin* _pluginEAC;
};

#endif /* ANGLEFROMDEVICE_H_ */
