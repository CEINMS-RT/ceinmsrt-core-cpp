/*
 * EMG0plugin.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: gdurandau
 */

#include "EMGFilepluginFilt.h"
#include <ProducersPluginVirtual.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <csignal>
#ifdef UNIX
#include <unistd.h>
#endif

EMGFileplugin::EMGFileplugin()
{
	// TODO Auto-generated constructor stub
	_verbose = 1;
	_record = false;
}

EMGFileplugin::~EMGFileplugin()
{
#ifdef VERBOSE

	if ( _verbose > 1 )
		std::cout << "\033[1;31mEMG from file " << this << "\033[0m" << std::endl;

#endif
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

	ss << "/emg.sto";
	std::string EMGDataFilename ( ss.str() );

#ifdef UNIX
	pid_ = getpid();
#endif
	dataFromFile_ = new DataFromFile ( ss.str().c_str() );

	muscleNames_ = dataFromFile_->getColumnNames();

	// Subject specific XML
	std::auto_ptr<NMSmodelType> subjectPointer;

	try
	{
		subjectPointer = std::auto_ptr<NMSmodelType> ( subject ( xmlName, xml_schema::flags::dont_initialize ) );
	}
	catch ( const xml_schema::exception& e )
	{
		if ( _verbose > 0 )
			COUT << e << endl;

		exit ( EXIT_FAILURE );
	}

	NMSmodelType::Channels_type& channels ( subjectPointer->Channels() );
	ChannelsType::Channel_sequence& channelSequence ( channels.Channel() );

	// Configuration XML
	std::auto_ptr<ExecutionType> executionPointer;

	try
	{
		std::auto_ptr<ExecutionType> temp ( execution ( executionName, xml_schema::flags::dont_initialize ) );
		executionPointer = temp;
	}
	catch ( const xml_schema::exception& e )
	{
		if ( _verbose > 0 )
			COUT << e << endl;

		exit ( EXIT_FAILURE );
	}

	// Get the EMG XML configuration
	const string& EMGFile = executionPointer->ConsumerPlugin().EMGDeviceFile().get();

	_executionEmgXml = new ExecutionEmgXml ( EMGFile );


	// Get filter parameters
	const std::vector<double>& aCoeffHP = _executionEmgXml->getACoeffHP();
	const std::vector<double>& bCoeffHP = _executionEmgXml->getBCoeffHP();
	const std::vector<double>& aCoeffLP = _executionEmgXml->getACoeffLP();
	const std::vector<double>& bCoeffLP = _executionEmgXml->getBCoeffLP();

	if ( aCoeffHP.size() == 0 || bCoeffHP.size() == 0 || aCoeffLP.size() == 0 || bCoeffLP.size() == 0 )
	{
		if ( _verbose > 0 )
			COUT << "Error: Filter coefficients size is zero." << std::endl;

		exit ( EXIT_FAILURE );
	}

	// Get Max EMG for normalization
	const std::vector<double>& maxAmp = _executionEmgXml->getMaxEmg();

	if ( maxAmp.size() == 0 )
	{
		if ( _verbose > 0 )
			COUT << "Error: Max EMG for Normalization size is zero." << std::endl;

		exit ( EXIT_FAILURE );
	}

	for ( ChannelsType::Channel_iterator it = channelSequence.begin(); it != channelSequence.end(); it++ )
	{
// 		nameSet_.insert ( it->name() );
		emgPreProcessingVect_.push_back (
			new EMGPreProcessing ( aCoeffLP, bCoeffLP, aCoeffHP, bCoeffHP,
					maxAmp[std::distance<ChannelsType::Channel_iterator> ( channelSequence.begin(), it )] ) );
	}

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
#ifdef VERBOSE

		if ( _verbose > 0 )
			cout << "\033[1;31mEMG: End of file\033[0m" << endl;

#endif
// 		kill ( pid_, SIGINT );
	}
	else
	{

		dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();
// 	dataFromFile_->readNextData();

		std::vector<double> data = dataFromFile_->getCurrentData();

		for ( std::vector<std::string>::const_iterator it = muscleNames_.begin(); it != muscleNames_.end(); it++ )
		{
// 	  COUT << data.at(std::distance<std::vector<std::string>::const_iterator>(muscleNames_.begin(), it)) << std::endl << std::flush;
			const int& cpt1 = std::distance<std::vector< std::string >::const_iterator> ( muscleNames_.begin(), it );
			mapData_[*it] = emgPreProcessingVect_[cpt1]->computeData ( data.at ( std::distance<std::vector<std::string>::const_iterator> ( muscleNames_.begin(), it ) ) );
		}
	}

	return mapData_;
}

const double& EMGFileplugin::getTime()
{
	return dataFromFile_->getCurrentTime();
}

extern "C" ProducersPluginVirtual* create()
{
	return new EMGFileplugin;
}

extern "C" void destroy ( ProducersPluginVirtual* p )
{
	delete p;
}
