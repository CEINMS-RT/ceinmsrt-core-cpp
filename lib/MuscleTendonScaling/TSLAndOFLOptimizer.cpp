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

#include "TSLAndOFLOptimizer.h"

TSLAndOFLOptimizer::TSLAndOFLOptimizer ( const std::string& xmlModelFile, MuscleTendonScalingStruct& mtss ) :
	_subjectPointer ( subject ( xmlModelFile.c_str(), xml_schema::flags::dont_initialize ) ), _mtss ( mtss )
{
	createCurves();
	setNumParameters ( 2 );
}

TSLAndOFLOptimizer::~TSLAndOFLOptimizer()
{

}

void TSLAndOFLOptimizer::createCurves()
{

	NMSmodelType::muscleDefault_type& muscleDefault ( _subjectPointer->muscleDefault() );
	MuscleDefaultType::Curve_sequence& curveSequence ( muscleDefault.Curve() );

	for ( MuscleDefaultType::Curve_iterator i = curveSequence.begin(); i != curveSequence.end(); ++i )
	{
		// each i is a curve
		std::string curveName = ( *i ).name();
		vector<double> x;

		PointsSequenceType xPoints = ( *i ).xPoints();
		PointsSequenceType::iterator pointsIt;

		for ( pointsIt = xPoints.begin(); pointsIt != xPoints.end(); ++pointsIt )
		{
			double currentX = ( *pointsIt );
			x.push_back ( currentX );
		}

		vector<double> y;

		PointsSequenceType yPoints = ( *i ).yPoints();

		for ( pointsIt = yPoints.begin(); pointsIt != yPoints.end(); ++pointsIt )
		{
			double currentY = ( *pointsIt );
			y.push_back ( currentY );
		}

		if ( curveName == "activeForceLength" )  _activeForceLengthCurve.resetPointsWith ( x, y );

		if ( curveName == "passiveForceLength" ) _passiveForceLengthCurve.resetPointsWith ( x, y );

		if ( curveName == "forceVelocity" )      _forceVelocityCurve.resetPointsWith ( x, y );
	}
}
