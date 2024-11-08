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
