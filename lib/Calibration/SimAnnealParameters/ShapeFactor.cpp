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

#include <iostream>
using std::cout;
using std::endl;

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "ShapeFactor.h"
#include "NMSmodel.h"

// A, C1 and C2 are singulary set for each muscle

template <typename NMSmodelT>
ShapeFactor<NMSmodelT>
::ShapeFactor ( NMSmodelT& subject,
        vector<string>& dofToCalibrate )
	: subject_ ( subject )
{


	subject_.getMusclesIndexFromDofs ( musclesIndexToCalibrate_, dofToCalibrate );
//  cout << dofToCalibrate << endl;

//  cout << "musclesIndexToCalibrate_\n";
//  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
//    cout << musclesIndexToCalibrate_.at(i) << "  ";
//  cout << endl;



	// Now group the muscles based on their strengthCoefficients
	subject_.getGroupMusclesBasedOnStrengthCoefficientsFilteredByMusclesIndexList ( strengthCoefficientValues_, muscleGroups_, musclesIndexToCalibrate_ );
	//subject_.getGroupMusclesBasedOnStrengthCoefficients(strengthCoefficientValues_, muscleGroups_);
	noParameters_ =  (int) musclesIndexToCalibrate_.size(); // LsT, Lom, strength Coeff and shape factor
	SyncToolsCal::Shared::strengthCoeff = (unsigned) strengthCoefficientValues_.size();
	SyncToolsCal::Shared::strengthCoeffReady.notify();
}


template <typename NMSmodelT>
void ShapeFactor<NMSmodelT>
::getStartingVectorParameters ( vector<double>& x )
{

	x.resize ( noParameters_ );
	int indexCounter = 0;

	// Then we have the shapeFactor
	vector<double> shapeFactors;
	subject_.getShapeFactors ( shapeFactors );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		x.at ( indexCounter + i ) = shapeFactors.at ( musclesIndexToCalibrate_.at ( i ) );

	indexCounter += (int) musclesIndexToCalibrate_.size();

}


template <typename NMSmodelT>
void ShapeFactor<NMSmodelT>
::setUpperLowerBounds ( vector<double>& upperBounds, vector<double>& lowerBounds )
{

	upperBounds.resize ( noParameters_ );
	lowerBounds.resize ( noParameters_ );



	int indexCounter = 0;

	// Then we have the shapeFactor
	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
	{
		upperBounds.at ( indexCounter + i ) = -0.001; // shape factor A
		lowerBounds.at ( indexCounter + i ) = -2.999;
	}

	indexCounter += (int) musclesIndexToCalibrate_.size();


}


template <typename NMSmodelT>
void ShapeFactor<NMSmodelT>
::setVectorParameters ( const vector<double>& x )
{

	int indexCounter =0;

	// Then we have the shapeFactor
	vector<double> currentShapeFactors;
	subject_.getShapeFactors ( currentShapeFactors );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentShapeFactors.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter + i );

	subject_.setShapeFactors ( currentShapeFactors );
	indexCounter += (int) musclesIndexToCalibrate_.size();

}


template <typename NMSmodelT>
std::ostream& operator<< ( std::ostream& output, const ShapeFactor<NMSmodelT>& p )
{

	output << "The number of muscle groups is: " << p.strengthCoefficientValues_.size() << endl;

	for ( unsigned int i = 0; i <  p.strengthCoefficientValues_.size(); ++i )
	{
		output << "Value: "  << p.strengthCoefficientValues_.at ( i ) << endl;
		output << "Muscles: ";

		for ( unsigned int j = 0; j < p.muscleGroups_.at ( i ).size(); ++j )
			output << p.muscleGroups_.at ( i ).at ( j ) << " ";

		output << endl;
	}

	return output;
}
