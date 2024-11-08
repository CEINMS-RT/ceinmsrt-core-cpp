/*
 * EMG0plugin.h
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

#ifndef EMG1plugin_H_
#define EMG1plugin_H_

#include "ProducersPluginVirtual.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "DataFromFile.h"
#include "ExecutionEmgXml.h"
#include "EMGPreProcessing.h"
#include "execution.hxx"
#include "NMSmodel.hxx"

class EMGFileplugin: virtual public ProducersPluginVirtual
{
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
// 		set<string> nameSet_;
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
		ExecutionEmgXml* _executionEmgXml;
		std::vector<EMGPreProcessing*> emgPreProcessingVect_; //!< Vector of class for the EMG pre-processing
};



#endif /* EMG1plugin_H_ */
