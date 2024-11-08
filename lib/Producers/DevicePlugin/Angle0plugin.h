/*
 * Angle0plugin.h
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

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
