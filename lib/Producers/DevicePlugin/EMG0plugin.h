/*
 * EMG0plugin.h
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

#ifndef EMG0PLUGIN_H_
#define EMG0PLUGIN_H_

#include "ProducersPluginVirtual.h"
#include <time.h>
#ifdef WIN32
#include <windows.h>
#endif
#ifdef UNIX
	#include <sys/time.h>
#endif
#include <stdio.h>
#include "NMSmodel.hxx"
#include <CommonCEINMS.h>

#include <iostream>

#include <getTime.h>

#ifdef WIN32
	class __declspec(dllexport) EMG0plugin : public ProducersPluginVirtual {
#endif
#ifdef UNIX
	class EMG0plugin : public ProducersPluginVirtual {
#endif
public:
	EMG0plugin();
	virtual ~EMG0plugin();
	void init(string xmlName, string executionName);

	const map<string, double>& GetDataMap() //< For having Data and name correspondence
	{
		timeNow_ = rtb::getTime();

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
	timeval tInit_;
	double timeNow_;
	map<string, double> _torque;
};



#endif /* EMG0PLUGIN_H_ */
