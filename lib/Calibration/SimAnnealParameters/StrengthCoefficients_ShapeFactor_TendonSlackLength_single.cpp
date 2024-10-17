// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox".
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software,
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS.
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621,
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com


#include <iostream>
using std::cout;
using std::endl;

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "StrengthCoefficients_ShapeFactor_TendonSlackLength_single.h"
#include "NMSmodel.h"

// A, C1 and C2 are singulary set for each muscle

template <typename NMSmodelT>
StrengthCoefficients_ShapeFactor_TendonSlackLength_single<NMSmodelT>
::StrengthCoefficients_ShapeFactor_TendonSlackLength_single ( NMSmodelT& subject,
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
	noParameters_ = (int) (musclesIndexToCalibrate_.size() + strengthCoefficientValues_.size() + musclesIndexToCalibrate_.size() + musclesIndexToCalibrate_.size()); // LsT, Lom, strength Coeff and shape factor
	SyncToolsCal::Shared::strengthCoeff = (unsigned) strengthCoefficientValues_.size();
	SyncToolsCal::Shared::strengthCoeffReady.notify();
}


template <typename NMSmodelT>
void StrengthCoefficients_ShapeFactor_TendonSlackLength_single<NMSmodelT>
::getStartingVectorParameters ( vector<double>& x )
{

	x.resize ( noParameters_ );
	int indexCounter;

	// at the beginning the strength coefficients
	for ( int i = 0; i < strengthCoefficientValues_.size(); ++i )
		x.at ( i ) = strengthCoefficientValues_.at ( i );

	indexCounter = (int) strengthCoefficientValues_.size();
	// Then we have the shapeFactor
	vector<double> shapeFactors;
	subject_.getShapeFactors ( shapeFactors );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		x.at ( indexCounter + i ) = shapeFactors.at ( musclesIndexToCalibrate_.at ( i ) );

	indexCounter += (int) musclesIndexToCalibrate_.size();

	// then, we ask the list of tendonSlackLengths
	vector<double> tendonSlackLengths;
	subject_.getTendonSlackLengths ( tendonSlackLengths );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		x.at ( indexCounter + i ) = tendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) );

	// then, we ask the list of optimalFiberLengths
	vector<double> optimalFiberLengths;
	subject_.getOptimalFiberLengths ( optimalFiberLengths );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		x.at ( indexCounter + musclesIndexToCalibrate_.size() + i ) = optimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) );
}


template <typename NMSmodelT>
void StrengthCoefficients_ShapeFactor_TendonSlackLength_single<NMSmodelT>
::setUpperLowerBounds ( vector<double>& upperBounds, vector<double>& lowerBounds )
{

	upperBounds.resize ( noParameters_ );
	lowerBounds.resize ( noParameters_ );

	// at the beginning the strength coefficients
	for ( int i = 0; i < strengthCoefficientValues_.size(); ++i )
	{
		upperBounds.at ( i ) = 1.5;
		lowerBounds.at ( i ) = 0.5;

	}

	int indexCounter = (int) strengthCoefficientValues_.size();

	// Then we have the shapeFactor
	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
	{
		upperBounds.at ( indexCounter + i ) = -0.001; // shape factor A
		lowerBounds.at ( indexCounter + i ) = -2.999;
	}

	indexCounter += (int) musclesIndexToCalibrate_.size();

	// then, tendonSlackLengths
	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
	{
		// :TODO: c'era
		// (RESTING_TENDON_LENGTH) + ((RESTING_TENDON_LENGTH)*0.15);
		// verificare che il restingTendonLength, sia il tendonSlackLength
		// then, we ask the list of tendonSlackLengths
		vector<double> tendonSlackLengths;
		subject_.getTendonSlackLengths ( tendonSlackLengths );
		upperBounds.at ( indexCounter + i ) =  tendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) ) + ( tendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) ) * 0.05 );
		lowerBounds.at ( indexCounter + i ) =  tendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) ) - ( tendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) ) * 0.05 );
	}

	// then, optimalFiberLengths
	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
	{
		vector<double> optimalFiberLengths;
		subject_.getOptimalFiberLengths ( optimalFiberLengths );
		upperBounds.at ( indexCounter + musclesIndexToCalibrate_.size() + i ) =  optimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) ) + ( optimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) ) * 0.025 );
		lowerBounds.at ( indexCounter + musclesIndexToCalibrate_.size() + i ) =  optimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) ) - ( optimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) ) * 0.025 );
	}

}


template <typename NMSmodelT>
void StrengthCoefficients_ShapeFactor_TendonSlackLength_single<NMSmodelT>
::setVectorParameters ( const vector<double>& x )
{

	// first we set the strength coefficients
	vector<double> currentStrengthCoefficientValues;
	int indexCounter = (int) strengthCoefficientValues_.size();
	currentStrengthCoefficientValues.resize ( strengthCoefficientValues_.size() );

	for ( int i = 0; i <  strengthCoefficientValues_.size(); ++i )
		currentStrengthCoefficientValues.at ( i ) = x.at ( i );

	subject_.setStrengthCoefficientsBasedOnGroups ( currentStrengthCoefficientValues, muscleGroups_ );

	// Then we have the shapeFactor
	vector<double> currentShapeFactors;
	subject_.getShapeFactors ( currentShapeFactors );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentShapeFactors.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter + i );

	subject_.setShapeFactors ( currentShapeFactors );
	indexCounter += (int) musclesIndexToCalibrate_.size();

//cout << "bp 4\n";
	// then we set the list of tendonSlackLengths
	vector<double> currentTendonSlackLengths;
	subject_.getTendonSlackLengths ( currentTendonSlackLengths );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentTendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter + i );

	subject_.setTendonSlackLengths ( currentTendonSlackLengths );
	
		vector<double> currentOptimalFiberLengths;
	subject_.getOptimalFiberLengths ( currentOptimalFiberLengths );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentOptimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter + musclesIndexToCalibrate_.size() + i );

	subject_.setOptimalFiberLengths ( currentOptimalFiberLengths );

}


template <typename NMSmodelT>
std::ostream& operator<< ( std::ostream& output, const StrengthCoefficients_ShapeFactor_TendonSlackLength_single<NMSmodelT>& p )
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
