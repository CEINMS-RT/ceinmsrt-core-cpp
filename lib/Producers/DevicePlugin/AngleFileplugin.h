/*
 * EMG0plugin.h
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

#ifndef EMG0PLUGIN_H_
#define EMG0PLUGIN_H_

#include "ProducersPluginVirtual.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "DataFromFile.h"
#include "time.h"

#ifdef WIN32
class __declspec(dllexport) AngleFileplugin : public ProducersPluginVirtual {
#endif
#ifdef UNIX
	class  AngleFileplugin : public ProducersPluginVirtual {
#endif
	public:
		AngleFileplugin();
		virtual ~AngleFileplugin();
		void init ( string executionXMLFile, string subjectCEINMSXMLFile );

		const map<string, double>& GetDataMap();

		const double& getTime();
		
		void reset()
		{
			
		}

		void stop()
		{
			delete dataFromFile_;

#ifdef VERBOSE

			if ( _verbose > 1 )
				std::cout << "\033[1;32mAngle read File thread end\033[0m" << std::endl;

#endif
		}

		void setDirectories ( std::string outDirectory, std::string inDirectory = std::string() )
		{
			_outDirectory = outDirectory;
			_inDirectory = inDirectory;
		}

		void setVerbose ( int verbose )
		{
			_verbose = verbose;
		}

		void setRecord ( bool record )
		{
			_record = record;
		}
		
		const map<string, double>& GetDataMapTorque()
		{
			return _torque;
		}

	protected:
		map<string, double> mapData_;
		set<string> nameSet_;
		DataFromFile* dataFromFile_;
		vector<string> dofName_;
#ifdef UNIX
		pid_t pid_;
#endif
		std::string _outDirectory;
		std::string _inDirectory;
		bool _record;
		int _verbose;
		map<string, double> _torque;
};



#endif /* EMG0PLUGIN_H_ */
