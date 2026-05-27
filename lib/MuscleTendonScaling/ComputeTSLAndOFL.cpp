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

#include "ComputeTSLAndOFL.h"

ComputeTSLAndOFL::ComputeTSLAndOFL ( const std::string& xmlModelFile, MTSS& mtss ) :
	_mtss ( mtss ), _xmlModelFile ( xmlModelFile )
{

}

ComputeTSLAndOFL::~ComputeTSLAndOFL()
{

}

void ComputeTSLAndOFL::run()
{
	for ( MTSS::iterator itMtss = _mtss.begin(); itMtss != _mtss.end(); itMtss++ )
	{
		SimTK::Real f = SimTK::NaN;

		TSLAndOFLOptimizer tsaofl ( _xmlModelFile, *itMtss );

		SimTK::Vector controls ( 2, 1.0 );
		SimTK::Vector lowerBounds ( 2 , 0.02 );
		SimTK::Vector upperBounds ( 2, 1.0 );

		lowerBounds[0] = 0.0;//itMtss->unscaledTendonSlackLength - itMtss->unscaledTendonSlackLength * 0.5;
		lowerBounds[1] = 0.02;//itMtss->unscaledOptimalFiberLength - itMtss->unscaledOptimalFiberLength * 0.5;
		
		upperBounds[0] = 1.0;//itMtss->unscaledTendonSlackLength + itMtss->unscaledTendonSlackLength * 0.5;
		upperBounds[1] = 1.0;//itMtss->unscaledOptimalFiberLength + itMtss->unscaledOptimalFiberLength * 0.5;
		
		controls[0] = itMtss->unscaledTendonSlackLength;
		controls[1] = itMtss->unscaledOptimalFiberLength;

		tsaofl.setParameterLimits ( lowerBounds, upperBounds );

		SimTK::Optimizer opt ( tsaofl, SimTK::InteriorPoint );

		opt.setConvergenceTolerance ( 1e-8 );
		opt.useNumericalGradient ( true );
		opt.useNumericalJacobian( true );
		opt.setMaxIterations ( 100000 );
		opt.setLimitedMemoryHistory ( 10000 );
		opt.setDifferentiatorMethod(SimTK::Differentiator::CentralDifference);

		f = opt.optimize ( controls );

		itMtss->scaledTendonSlackLength = controls[0];
		itMtss->scaledOptimalFiberLength = controls[1];
	}
}
