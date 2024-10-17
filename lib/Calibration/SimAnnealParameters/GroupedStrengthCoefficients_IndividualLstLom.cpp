// This is part of
// NeuroMuscoloSkeletal Model Software (NMS)
// Copyright (C) 2010 David Lloyd Massimo Sartori Monica Reggiani
//
// ?? Licenza ??
//
// The authors may be contacted via:
// email: massimo.srt@gmail.com monica.reggiani@gmail.com



#include <iostream>
using std::cout;
using std::endl;

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "NMSmodel.h"
#include "GroupedStrengthCoefficients_IndividualLstLom.h"

// A, C1 and C2 are global. TendonSlackLength and OptimalFiberLength are individually set for each muscle. StrengthCoefficients are individually set for groups of MTUs
template<typename NMSmodelT>
GroupedStrengthCoefficients_IndividualLstLom<NMSmodelT>::
GroupedStrengthCoefficients_IndividualLstLom ( NMSmodelT& subject, const vector<string>& dofToCalibrate ) : subject_ ( subject )
{


	subject_.getMusclesIndexFromDofs ( musclesIndexToCalibrate_, dofToCalibrate );

	// Now group the muscles based on their strengthCoefficients
	subject_.getGroupMusclesBasedOnStrengthCoefficientsFilteredByMusclesIndexList ( strengthCoefficientValues_, muscleGroups_, musclesIndexToCalibrate_ );
	//subject_.getGroupMusclesBasedOnStrengthCoefficients(strengthCoefficientValues_, muscleGroups_);
	noParameters_ = (int) (strengthCoefficientValues_.size() + musclesIndexToCalibrate_.size() * 2 + 1); // Muscle froce, ofl, tsl, shape factor
	SyncToolsCal::Shared::strengthCoeff = (unsigned) strengthCoefficientValues_.size();
	SyncToolsCal::Shared::strengthCoeffReady.notify();
}

template<typename NMSmodelT>
void GroupedStrengthCoefficients_IndividualLstLom<NMSmodelT>::
getStartingVectorParameters ( vector<double>& x )
{

	x.resize ( noParameters_ );
	int indexCounter;

	// at the beginning we place the strength coefficients
	for ( int i = 0; i < strengthCoefficientValues_.size(); ++i )
		x.at ( i ) = strengthCoefficientValues_.at ( i );

	indexCounter = (int) strengthCoefficientValues_.size();

// 	and the A (shape) factor
	vector<double> shapeCoefficients;
	subject_.getShapeFactors ( shapeCoefficients );
	x.at ( indexCounter ) = shapeCoefficients.at ( 0 );
	indexCounter += 1;

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

template<typename NMSmodelT>
void GroupedStrengthCoefficients_IndividualLstLom<NMSmodelT>::setUpperLowerBounds (
	vector<double>& upperBounds, vector<double>& lowerBounds )
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

// 	/*//and then the shape factor A
	upperBounds.at ( indexCounter ) = -0.001; // shape factor A
	lowerBounds.at ( indexCounter ) = -2.999;
	indexCounter += 1;

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

template<typename NMSmodelT>
void GroupedStrengthCoefficients_IndividualLstLom<NMSmodelT>::setVectorParameters ( const vector<double>& x )
{

	// first we set the strength coefficients
	vector<double> currentStrengthCoefficientValues;
	int indexCounter = (int) strengthCoefficientValues_.size();
	currentStrengthCoefficientValues.resize ( strengthCoefficientValues_.size() );

	for ( int i = 0; i <  strengthCoefficientValues_.size(); ++i )
		currentStrengthCoefficientValues.at ( i ) = x.at ( i );

	subject_.setStrengthCoefficientsBasedOnGroups ( currentStrengthCoefficientValues, muscleGroups_ );

// 	// and then the shape factor A
	vector<double> currentShapeCoefficients;
	subject_.getShapeFactors ( currentShapeCoefficients );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentShapeCoefficients.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter );

	subject_.setShapeFactors ( currentShapeCoefficients );
	indexCounter += 1;

//cout << "bp 4\n";
	// then we set the list of tendonSlackLengths
	vector<double> currentTendonSlackLengths;
	subject_.getTendonSlackLengths ( currentTendonSlackLengths );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentTendonSlackLengths.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter + i );

	subject_.setTendonSlackLengths ( currentTendonSlackLengths );

	// cout << "The set of parameters is: ";
	//for (unsigned int i = 0; i < x.size(); ++i)
	//cout << x.at(i) << " " ;
	//cout << endl;
	//cout << "The subject currently is: ";
	//cout << subject_ << endl;

	// then we set the list of optimalFiberLengths
	vector<double> currentOptimalFiberLengths;
	subject_.getOptimalFiberLengths ( currentOptimalFiberLengths );

	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i )
		currentOptimalFiberLengths.at ( musclesIndexToCalibrate_.at ( i ) ) = x.at ( indexCounter + musclesIndexToCalibrate_.size() + i );

	subject_.setOptimalFiberLengths ( currentOptimalFiberLengths );
}


template<typename NMSmodelT>
std::ostream& operator<< ( std::ostream& output, const GroupedStrengthCoefficients_IndividualLstLom<NMSmodelT>& p )
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

//#include "ParamGroupedScIndLSTLOMGlobC1C2APolicyTemplates.h"
