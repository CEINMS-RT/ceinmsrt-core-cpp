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

#ifndef MUSCLETENDONSCALING_H
#define MUSCLETENDONSCALING_H

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>

#include "ComputePostureControlPoints.h"
#include "ComputeMuscleTendonLength.h"
#include "ComputeMuscleLength.h"
#include "ComputeTSLAndOFL.h"
#include <cmath>

class MuscleTendonScaling
{
	public:
		MuscleTendonScaling(const std::string& unscaledOsimModelName, const std::string& scaledOsimModelName,
							const std::string& configurationFile, const std::string& translateFileName);
		~MuscleTendonScaling();
		void run();
		double getTendonSlackLength(const std::string& muscleName);
		double getOptimalFiberLength(const std::string& muscleName);
		
		inline void setVerbose(const int& verbose)
		{
			_verbose = verbose;
		}
		
	protected:
		OpenSim::Model _unscaledModel;
		OpenSim::Model _scaledModel;
		MTSS _mtss;
		ComputePostureControlPoints* _cpcp;
		ComputeMuscleTendonLength* _cmtl;
		ComputeMuscleLength* _cml;
		ComputeTSLAndOFL* _ctslofl;
		const std::string& _configurationFile;
		const std::string& _translateFileName;
		bool _run;
		int _verbose;
};

#endif

#endif // MUSCLETENDONSCALING_H
