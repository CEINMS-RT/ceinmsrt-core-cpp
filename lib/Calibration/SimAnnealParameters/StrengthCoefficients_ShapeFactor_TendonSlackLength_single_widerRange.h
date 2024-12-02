// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software, regulated by the license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots"
//

#ifndef StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange_widerRange_h
#define StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange_widerRange_h

#include <iostream>
#include <string>
#include <vector>

template<typename NMSmodelT>
class StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange;

template<typename NMSmodelT>
std::ostream& operator<< (std::ostream& output, 
                const StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<NMSmodelT>& p);


template <typename NMSmodelT>
class StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange {
public:
	StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange(NMSmodelT& subject, std::vector< std::string >& dofToCalibrate);
    // the number of parameters is strength coeffs (3) + tendon slack lengths (= no Muscles) + SlackLength + C1 + C2
    int getNoParameters() { return noParameters_; }
    void getStartingVectorParameters(std::vector<double>& x);
    void setVectorParameters(const std::vector<double>& x);
    void setUpperLowerBounds(std::vector<double>& upperBounds, std::vector<double>& lowerBounds);
    friend std::ostream& operator<< <> (std::ostream& output, 
                  const StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange& p);
private:
    NMSmodelT& subject_;
    std::vector<double> strengthCoefficientValues_;
    std::vector<unsigned int> musclesIndexToCalibrate_;
    std::vector< std::vector<int> > muscleGroups_; // each index is a vector of id for the muscles
    int noParameters_;
};

#include "StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange.cpp"

#endif
