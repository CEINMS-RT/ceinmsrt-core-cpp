/*
 * Copyright (c) 2015, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
