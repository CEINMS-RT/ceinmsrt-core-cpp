/*
 * AngleFromDevice.h
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

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
