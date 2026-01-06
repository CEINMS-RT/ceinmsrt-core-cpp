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

#include "AngleFileplugin.h"
#include <ProducersPluginVirtual.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <csignal>
#ifdef UNIX
	#include <unistd.h>
#endif

AngleFileplugin::AngleFileplugin()
{
	// TODO Auto-generated constructor stub
	_verbose = 1;
	_record = false;
}

AngleFileplugin::~AngleFileplugin()
{
}

void AngleFileplugin::init ( std::string executionXMLFile, std::string subjectCEINMSXMLFile )
{
#ifdef VERBOSE

	if ( _verbose > 1 )
		std::cout << "init Angle lib" << std::endl;

#endif

	std::stringstream ss;

	if ( !_inDirectory.empty() )
	{
		ss << "./";
		ss << _inDirectory;
	}
	else
	{
		ss << ".";
	}

	ss << "/ik.sto";

	dataFromFile_ = new DataFromFile ( ss.str().c_str() );

	dofName_ = dataFromFile_->getColumnNames( );

#ifdef VERBOSE

	if ( _verbose > 1 )
		cout << "Plugin AngleFileplugin, initialisation done." << endl;

#endif
}

const map<string, double>& AngleFileplugin::GetDataMap()
{
	// 	first we build a vector of emg read from the file

	if ( !dataFromFile_->areStillData()  )
	{
		if ( _verbose > 0 )
			cout << "\033[1;31mAngle: End of file\033[0m" << endl;

// 		kill ( pid_, SIGINT );
	}
	else
	{
// 	dataFromFile_->readNextData();
		dataFromFile_->readNextData();


		std::vector<double> data = dataFromFile_->getCurrentData();

		for ( std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++ )
		{
			mapData_[*it] = data.at ( std::distance<std::vector<std::string>::const_iterator> ( dofName_.begin(), it ) );
		}
	}

// 	if(_process)


	return mapData_;
}

const double& AngleFileplugin::getTime()
{
	return dataFromFile_->getCurrentTime();
}

#ifdef UNIX
extern "C" ProducersPluginVirtual* create() {
	return new AngleFileplugin;
}

extern "C" void destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif

#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) ProducersPluginVirtual* __cdecl create() {
	return new AngleFileplugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif
