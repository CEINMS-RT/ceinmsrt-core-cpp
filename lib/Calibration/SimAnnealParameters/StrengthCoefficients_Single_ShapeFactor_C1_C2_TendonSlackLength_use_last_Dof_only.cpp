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

#include "NMSmodel.h"

// A, C1, C2 and tendon slack lengths are singulary set for each muscle
// consider the parameter of the last entered DoF only

template<typename Activation, typename Tendon>
StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only<Activation, Tendon >::
         StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only(NMSmodelT& subject, vector<string>& dofToCalibrate)
:subject_(subject) {

  
  subject_.getMusclesIndexFromLastDof(musclesIndexToCalibrate_, dofToCalibrate);

//  cout << "musclesIndexToCalibrate_\n";
//  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
//    cout << musclesIndexToCalibrate_.at(i) << "  ";
//  cout << endl;
  
  // Now group the muscles based on their strengthCoefficients
  subject_.getGroupMusclesBasedOnStrengthCoefficientsFilteredByMusclesIndexList(strengthCoefficientValues_, muscleGroups_, musclesIndexToCalibrate_);
  //subject_.getGroupMusclesBasedOnStrengthCoefficients(strengthCoefficientValues_, muscleGroups_);
  noParameters_ = 3*musclesIndexToCalibrate_.size() + strengthCoefficientValues_.size() + musclesIndexToCalibrate_.size();
}


template<typename Activation, typename Tendon>
void StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only<Activation, Tendon >::
        getStartingVectorParameters(vector<double>& x) {
        
  x.resize(noParameters_);
  int indexCounter;
  // at the beginning the strength coefficients
  for ( int i = 0; i < strengthCoefficientValues_.size(); ++i )
    x.at(i) = strengthCoefficientValues_.at(i);
  indexCounter = strengthCoefficientValues_.size();
  // Then we have the shapeFactor
  vector<double> shapeFactors;
  subject_.getShapeFactors(shapeFactors);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    x.at(indexCounter+i) = shapeFactors.at(musclesIndexToCalibrate_.at(i)); 
  indexCounter += musclesIndexToCalibrate_.size();

    // the C1
  vector<double> c1Coefficients;
  subject_.getC1Coefficients(c1Coefficients);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    x.at(indexCounter+i) = c1Coefficients.at(musclesIndexToCalibrate_.at(i)); 
  indexCounter += musclesIndexToCalibrate_.size();

    // and the C2

  vector<double> c2Coefficients;
  subject_.getC2Coefficients(c2Coefficients);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    x.at(indexCounter+i) = c2Coefficients.at(musclesIndexToCalibrate_.at(i)); 
  indexCounter += musclesIndexToCalibrate_.size();
  
  // then, we ask the list of tendonSlackLengths
  vector<double> tendonSlackLengths;
  subject_.getTendonSlackLengths(tendonSlackLengths);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    x.at(indexCounter+i) = tendonSlackLengths.at(musclesIndexToCalibrate_.at(i)); 
}


template<typename Activation, typename Tendon>    
void StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only<Activation, Tendon >::setUpperLowerBounds(
            vector<double>& upperBounds, vector<double>& lowerBounds) {
       
  upperBounds.resize(noParameters_); 
  lowerBounds.resize(noParameters_);

  // at the beginning the strength coefficients
  for ( int i = 0; i < strengthCoefficientValues_.size(); ++i ) {
    upperBounds.at(i) = 2.5;
    lowerBounds.at(i) = 0.5;
    
  }  
  int indexCounter = strengthCoefficientValues_.size();
  // Then we have the shapeFactor
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
  {
      upperBounds.at(indexCounter+i) = 0.3; // shape factor A 
      lowerBounds.at(indexCounter+i) = 0.0000001;	
  }
  indexCounter += musclesIndexToCalibrate_.size();

  // the C1

  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
  {
  upperBounds.at(indexCounter+i) = -0.005;
  lowerBounds.at(indexCounter+i) = -0.9950;
  }
  indexCounter += musclesIndexToCalibrate_.size();
  
// and the C2
   for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
  {
  upperBounds.at(indexCounter+i) = -0.0050;
  lowerBounds.at(indexCounter+i) = -0.9950;
  }
  indexCounter += musclesIndexToCalibrate_.size();
  
  // then, tendonSlackLengths
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i) {
  // :TODO: c'era
  // (RESTING_TENDON_LENGTH) + ((RESTING_TENDON_LENGTH)*0.15); 
  // verificare che il restingTendonLength, sia il tendonSlackLength
  // then, we ask the list of tendonSlackLengths
  vector<double> tendonSlackLengths;
  subject_.getTendonSlackLengths(tendonSlackLengths);
    upperBounds.at(indexCounter+i) =  tendonSlackLengths.at(musclesIndexToCalibrate_.at(i)) + (tendonSlackLengths.at(musclesIndexToCalibrate_.at(i))*0.15); 
    lowerBounds.at(indexCounter+i) =  tendonSlackLengths.at(musclesIndexToCalibrate_.at(i)) - (tendonSlackLengths.at(musclesIndexToCalibrate_.at(i))*0.15);
  }
   	
}
 
 
 template<typename Activation, typename Tendon>
void StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only<Activation, Tendon >::setVectorParameters(const vector<double>& x) {
	
		// first we set the strength coefficients
	vector<double> currentStrengthCoefficientValues;
	int indexCounter = strengthCoefficientValues_.size();
	currentStrengthCoefficientValues.resize(strengthCoefficientValues_.size());
	
	for (int i = 0; i <  strengthCoefficientValues_.size(); ++i)
		currentStrengthCoefficientValues.at(i) = x.at(i);
	
	subject_.setStrengthCoefficientsBasedOnGroups(currentStrengthCoefficientValues, muscleGroups_);
  
		// Then we have the shapeFactor
  vector<double> currentShapeFactors;
  subject_.getShapeFactors(currentShapeFactors);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    currentShapeFactors.at(musclesIndexToCalibrate_.at(i)) = x.at(indexCounter+i);
	subject_.setShapeFactors(currentShapeFactors);	
  indexCounter += musclesIndexToCalibrate_.size();

//cout << "bp 2\n";
  	// the C1
  vector<double> currentC1Coefficients;
  subject_.getC1Coefficients(currentC1Coefficients);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    currentC1Coefficients.at(musclesIndexToCalibrate_.at(i)) = x.at(indexCounter+i);
	subject_.setC1Coefficients(currentC1Coefficients);	
  indexCounter += musclesIndexToCalibrate_.size();

//cout << "bp 3\n";
		// and the C2
  vector<double> currentC2Coefficients;
  subject_.getC2Coefficients(currentC2Coefficients);
  for (int i = 0; i < musclesIndexToCalibrate_.size(); ++i)
    currentC2Coefficients.at(musclesIndexToCalibrate_.at(i)) = x.at(indexCounter+i);
	subject_.setC2Coefficients(currentC2Coefficients);	
  indexCounter += musclesIndexToCalibrate_.size();
	
//cout << "bp 4\n";
	  // then we set the list of tendonSlackLengths
	vector<double> currentTendonSlackLengths;
  subject_.getTendonSlackLengths(currentTendonSlackLengths);
	for ( int i = 0; i < musclesIndexToCalibrate_.size(); ++i ) 
	  currentTendonSlackLengths.at(musclesIndexToCalibrate_.at(i)) = x.at(indexCounter+i);
	subject_.setTendonSlackLengths(currentTendonSlackLengths);

    // cout << "The set of parameters is: ";
    //for (unsigned int i = 0; i < x.size(); ++i)
    //cout << x.at(i) << " " ;
    //cout << endl;
    //cout << "The subject currently is: ";
    //cout << subject_ << endl;
}

template<typename Activation, typename Tendon>
std::ostream& operator<< (std::ostream& output, const StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only<Activation, Tendon >& p)
{
 output << "The number of muscle groups is: " << p.strengthCoefficientValues_.size() << endl;
 for ( unsigned int i = 0; i <  p.strengthCoefficientValues_.size(); ++i ) {
   output << "Value: "  << p.strengthCoefficientValues_.at(i) << endl;
   output << "Muscles: ";
   for ( unsigned int j = 0; j < p.muscleGroups_.at(i).size(); ++j )
      output << p.muscleGroups_.at(i).at(j) << " ";
   output << endl;   
 }
    
 return output;
}
