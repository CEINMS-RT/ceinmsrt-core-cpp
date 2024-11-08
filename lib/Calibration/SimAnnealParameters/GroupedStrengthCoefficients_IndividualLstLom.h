// This is part of
// NeuroMuscoloSkeletal Model Software (NMS)
// Copyright (C) 2010 David Lloyd Massimo Sartori Monica Reggiani
//
// ?? Licenza ??
//
// The authors may be contacted via:
// email: massimo.srt@gmail.com monica.reggiani@gmail.com

#ifndef GroupedStrengthCoefficients_IndividualLstLom_h
#define GroupedStrengthCoefficients_IndividualLstLom_h

#include <iostream>
#include <string>
#include <vector>
#include "NMSmodel.h"

template<typename NMSmodelT>
class GroupedStrengthCoefficients_IndividualLstLom;

template<typename NMSmodelT>
std::ostream& operator<< (std::ostream& output, 
                const GroupedStrengthCoefficients_IndividualLstLom<NMSmodelT>& p);

template<typename NMSmodelT>
class GroupedStrengthCoefficients_IndividualLstLom {
public:
	GroupedStrengthCoefficients_IndividualLstLom(NMSmodelT& subject, const std::vector< std::string >& dofToCalibrate);
	// the number of parameters is: grouped strength coeffs (= no Groups) + tendon slack lengths (= no Muscles) + optimal fiber lengths (= no Muscles) + C1 (= blobal) + C2 (= global) + A (= global)
	int getNoParameters() { return noParameters_; }
	void getStartingVectorParameters(std::vector<double>& x);
	//void checkConstraint(std::vector<double>& x);
	void setVectorParameters(const std::vector<double>& x);
	void setUpperLowerBounds(std::vector<double>& upperBounds, std::vector<double>& lowerBounds);
	friend std::ostream& operator<< <> (std::ostream& output, 
                  const GroupedStrengthCoefficients_IndividualLstLom& p);
private:

    // GroupedStrengthCoefficients_IndividualLstLom() {};
	NMSmodelT& subject_;
	std::vector<double> strengthCoefficientValues_;
    std::vector<unsigned int> musclesIndexToCalibrate_;
	std::vector< std::vector<int> > muscleGroups_; // each index is a vector of id for the muscles
 // std::vector< std::vector<int> > filteredMuscleGroups_;
	int noParameters_;
};

#include "GroupedStrengthCoefficients_IndividualLstLom.cpp"

#endif
