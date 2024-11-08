/*
 * EMG0plugin.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

#include "EMGFileplugin.h"
#include <ProducersPluginVirtual.h>
#include <execution.hxx>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <csignal>
#ifdef UNIX
	#include <unistd.h>
#endif

#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

EMGFileplugin::EMGFileplugin()
{
	// TODO Auto-generated constructor stub
	_verbose = 1;
	_record = false;
}

EMGFileplugin::~EMGFileplugin()
{

}

void EMGFileplugin::init ( string xmlName, string executionName )
{

#ifdef VERBOSE

	if ( _verbose > 1 )
		cout << "init EMG lib" << endl;

#endif

	std::stringstream ss;

	if ( !_inDirectory.empty() )
	{
		ss << "./";
		ss << _inDirectory;
	}
	else
		ss << ".";

	ss << "/emgFilt.sto";
	std::string EMGDataFilename ( ss.str() );
#ifdef UNIX
	pid_ = getpid();
#endif

	if(!fs::exists(fs::path(ss.str()))){
		std::unique_ptr<ExecutionType> executionPointer ( execution ( executionName, xml_schema::flags::dont_initialize ) );
		std::string emgFileName = executionPointer->ConsumerPlugin().EMGDeviceFile().get();
		if(_verbose){
			std::cout << ss.str() << " does not exist in the current folder. Switching to " << emgFileName << std::endl;
		}
		ss.str("");
		ss << emgFileName;

	}


	dataFromFile_ = new DataFromFile(ss.str().c_str());
	
	muscleNames_ = dataFromFile_->getColumnNames();

#ifdef VERBOSE

	if ( _verbose > 1 )
		cout << "Plugin EMGFileplugin, initialisation done." << endl;

#endif
}

const map<string, double>& EMGFileplugin::GetDataMap()
{
	// 	first we build a vector of emg read from the file

	mapData_.clear();

	if ( !dataFromFile_->areStillData() )
	{
		if ( _verbose > 0 )
			cout << "\033[1;31mEMG: End of file\033[0m" << endl;
	}
	else
	{
		dataFromFile_->readNextData();
	}
	// Angle data might not have yet ended. Returns the last data available anyway
	std::vector<double> data = dataFromFile_->getCurrentData();
	for ( std::vector<std::string>::const_iterator it = muscleNames_.begin(); it != muscleNames_.end(); it++ )
	{
		mapData_[*it] = data.at(std::distance<std::vector<std::string>::const_iterator>(muscleNames_.begin(), it));
	}
	return mapData_;
}

const double& EMGFileplugin::getTime()
{
	return dataFromFile_->getCurrentTime();
}

#ifdef UNIX
extern "C" ProducersPluginVirtual* create() {
	return new EMGFileplugin;
}

extern "C" void destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif
#ifdef WIN32
extern "C" __declspec (dllexport) ProducersPluginVirtual* __cdecl create() {
	return new EMGFileplugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(ProducersPluginVirtual* p) {
	delete p;
}
#endif
