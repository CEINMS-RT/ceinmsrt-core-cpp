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
