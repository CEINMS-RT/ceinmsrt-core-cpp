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

#include "MuscleTendonScaling.h"

MuscleTendonScaling::MuscleTendonScaling ( const std::string& unscaledOsimModelName, const std::string& scaledOsimModelName,
		const std::string& configurationFile, const std::string& translateFileName ) :
	_unscaledModel ( OpenSim::Model ( unscaledOsimModelName ) ), _scaledModel ( OpenSim::Model ( scaledOsimModelName ) ), _configurationFile ( configurationFile ),
	_translateFileName ( translateFileName )
{
	_run = false;
	_verbose = 1;
}

MuscleTendonScaling::~MuscleTendonScaling()
{
	if ( _run )
	{
		delete _cpcp;
		delete _cmtl;
		delete _cml;
		delete _ctslofl;
	}
}

void MuscleTendonScaling::run()
{
	if ( _verbose > 1 )
		std::cout << "ComputePostureControlPoints" << std::endl;

	_cpcp = new ComputePostureControlPoints ( _configurationFile, _unscaledModel, _mtss, _translateFileName );

	if ( _verbose > 1 )
		std::cout << "ComputeMuscleTendonLength" << std::endl;

	_cmtl = new ComputeMuscleTendonLength ( _unscaledModel, _scaledModel, _mtss, _translateFileName );
	_cmtl->run();

	if ( _verbose > 1 )
		std::cout << "ComputeMuscleLength" << std::endl;

	_cml = new ComputeMuscleLength ( _configurationFile, _mtss );
	_cml->setVerbose(_verbose);
	_cml->run();

	if ( _verbose > 1 )
		std::cout << "ComputeTSLAndOFL" << std::endl;

	_ctslofl = new ComputeTSLAndOFL ( _configurationFile, _mtss );
	_ctslofl->run();
	_run = true;
}

double MuscleTendonScaling::getTendonSlackLength ( const std::string& muscleName )
{
	if ( _run )
	{
		for ( MTSS::const_iterator it = _mtss.begin(); it != _mtss.end(); it++ )
		{
			if ( it->muscleName == muscleName )
				return it->scaledTendonSlackLength;
		}

		//std::cerr << "Muscle: " << muscleName << " not found." <<  std::endl;
		return -1;
	}
	else
	{
		std::cerr << "Run need to be called before asking for getTendonSlackLength." << std::endl;
		exit ( 1 );
	}
}

double MuscleTendonScaling::getOptimalFiberLength ( const std::string& muscleName )
{
	if ( _run )
	{
		for ( MTSS::const_iterator it = _mtss.begin(); it != _mtss.end(); it++ )
		{
			if ( it->muscleName == muscleName )
				return it->scaledOptimalFiberLength;
		}

// 		std::cerr << "Muscle: " << muscleName << " not found." <<  std::endl;
		return -1;
	}
	else
	{
		std::cerr << "Run need to be called before asking for getOptimalFiberLength." << std::endl;
		exit ( 1 );
	}
}
