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

#ifndef EMGFROMDEVICE_H_
#define EMGFROMDEVICE_H_

#include "CommonCEINMS.h"
#include "DynLib.h"
#include "EMGFromX.h"
#include <map>
#include <vector>
#include <set>
#include <boost/unordered_map.hpp>
#include "execution.hxx"
#include <csignal>
#include <stddef.h>
#include "ExecutionXmlReader.h"
#include <ProducersPluginVirtual.h>

class EMGFromDevice: public DynLib<ProducersPluginVirtual>, public EMGFromX {
public:
	EMGFromDevice()  {}
	EMGFromDevice(string xmlName, string executionName, bool record = false, 
		std::string recordDirectory = std::string(), std::string processDirectory = std::string(), bool process = false);
	virtual ~EMGFromDevice();
	void operator()();
private:

// 	void getMusclesNames();
// 	void getMusclesNamesOnChannel();
	void computeChannelNameOnMuscle();

	typedef std::map<std::string, std::vector <std::string> > MapStrVectStr;
	typedef boost::unordered_map<std::string, std::vector <std::string> > UnMapStrVectStr;

	vector<string> muscleName_;
	MapStrVectStr musclesNamesOnChannel_;
	UnMapStrVectStr channelNameOnMuscle_;
	bool firstPass_;
	string xmlName_;
	string execName_;
	string osimFile_;
	string labFile_;
	int _verbose;
	bool _gui;
	bool _process;
	std::string _processDirectory;
	bool _record;
	std::string _recordDirectory;
};

#endif /* EMGFROMDEVICE_H_ */
