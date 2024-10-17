/*
 * EMG0plugin.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

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
