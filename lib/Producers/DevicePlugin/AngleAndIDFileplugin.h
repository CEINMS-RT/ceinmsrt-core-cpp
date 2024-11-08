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

#if defined _WIN32 || defined __CYGWIN__
//#ifdef BUILDING_DLL
#ifdef __GNUC__
#define DLL_PUBLIC __attribute__ ((dllexport))
#else
#define DLL_PUBLIC __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
#endif
#else
#ifdef __GNUC__
#define DLL_PUBLIC __attribute__ ((dllimport))
#else
#define DLL_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
#endif
#endif
#define DLL_LOCAL
/*#else
#if __GNUC__ >= 4
#define DLL_PUBLIC __attribute__ ((visibility ("default")))
#define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define DLL_PUBLIC
#define DLL_LOCAL
#endif
#endif*/

class AngleFileplugin: virtual public ProducersPluginVirtual
{
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
			delete idFromFile_;

			std::cout << "stop angle: " << this << std::endl << std::flush;
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
		DataFromFile* idFromFile_;
		vector<string> dofName_;
		vector<string> dofNameID_;
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
