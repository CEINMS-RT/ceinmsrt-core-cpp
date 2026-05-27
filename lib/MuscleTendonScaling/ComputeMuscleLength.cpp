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

#include "ComputeMuscleLength.h"

ComputeMuscleLength::ComputeMuscleLength ( const std::string& xmlModelFile, MTSS& mtss ) :
	_mtss ( mtss ), _xmlModelFile ( xmlModelFile )
{
	_verbose = 1;
}

ComputeMuscleLength::~ComputeMuscleLength()
{

}

void ComputeMuscleLength::run()
{
// 	std::vector<double> fb;
// 
// 	fb.push_back ( 1.0442 );
// 	fb.push_back ( 1.0175 );
// 	fb.push_back ( 0.9601 );
// 	fb.push_back ( 0.8980 );
// 	fb.push_back ( 0.8309 );
// 	fb.push_back ( 0.7605 );
// 	fb.push_back ( 0.6907 );
// 	fb.push_back ( 0.6252 );
// 	fb.push_back ( 0.5646 );
// 	fb.push_back ( 0.5072 );
// 	fb.push_back ( 0.4818 );

	for ( MTSS::iterator itMtss = _mtss.begin(); itMtss != _mtss.end(); itMtss++ )
		for ( int i = 0; i < 11; i++ )
		{

			if(_verbose > 2)
 			std::cout << "optimisation: " << itMtss->muscleName << " : " << i << std::endl;

			SimTK::Real f = SimTK::NaN;

			OptimalFiberLengthOptimizer oflo ( _xmlModelFile, *itMtss, i );

			SimTK::Vector controls ( 1, 1.0 );
			SimTK::Vector lowerBounds ( 1, 0.02 );
			SimTK::Vector upperBounds ( 1, 2.0 );

			oflo.setParameterLimits ( lowerBounds, upperBounds );

			SimTK::Optimizer opt ( oflo, SimTK::InteriorPoint );

			opt.setConvergenceTolerance ( 1e-7 );
			opt.useNumericalGradient ( true );
			opt.useNumericalJacobian ( true );
			opt.setMaxIterations ( 1000000 );
			opt.setLimitedMemoryHistory ( 10000 );
			opt.setDifferentiatorMethod ( SimTK::Differentiator::CentralDifference );

			f = opt.optimize ( controls );

			itMtss->fiberLength.push_back ( controls[0] );
		}
}
