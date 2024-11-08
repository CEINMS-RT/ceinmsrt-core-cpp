/*
 * EMGFromDevice.h
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

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
