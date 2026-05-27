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

#ifndef EMG1plugin_H_
#define EMG1plugin_H_

#include "ProducersPluginVirtual.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "DataFromFile.h"

#ifdef WIN32
class __declspec(dllexport) EMGFileplugin : public ProducersPluginVirtual {
#endif
#ifdef UNIX
	class  EMGFileplugin : public ProducersPluginVirtual {
#endif
	public:
		EMGFileplugin();
		virtual ~EMGFileplugin();
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
				std::cout << "\033[1;32mEMG read File thread end\033[0m" << std::endl;

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
		vector<string> muscleNames_;
		DataFromFile* dataFromFile_;
#ifdef UNIX
		pid_t pid_;
#endif
		std::string _outDirectory;
		std::string _inDirectory;
		bool _record;
		int _verbose;
		map<string, double> _torque;
};



#endif /* EMG1plugin_H_ */
