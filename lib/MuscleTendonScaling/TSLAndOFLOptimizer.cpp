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
