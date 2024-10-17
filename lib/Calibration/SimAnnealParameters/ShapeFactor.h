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

#ifndef ShapeFactor_h
#define ShapeFactor_h

#include <iostream>
#include <string>
#include <vector>

template<typename NMSmodelT>
class ShapeFactor;

template<typename NMSmodelT>
std::ostream& operator<< (std::ostream& output, 
                const ShapeFactor<NMSmodelT>& p);


template <typename NMSmodelT>
class ShapeFactor {
public:
	ShapeFactor(NMSmodelT& subject, std::vector< std::string >& dofToCalibrate);
    // the number of parameters is strength coeffs (3) + tendon slack lengths (= no Muscles) + SlackLength + C1 + C2
    int getNoParameters() { return noParameters_; }
    void getStartingVectorParameters(std::vector<double>& x);
    void setVectorParameters(const std::vector<double>& x);
    void setUpperLowerBounds(std::vector<double>& upperBounds, std::vector<double>& lowerBounds);
    friend std::ostream& operator<< <> (std::ostream& output, 
                  const ShapeFactor& p);
private:
    NMSmodelT& subject_;
    std::vector<double> strengthCoefficientValues_;
    std::vector<unsigned int> musclesIndexToCalibrate_;
    std::vector< std::vector<int> > muscleGroups_; // each index is a vector of id for the muscles
    int noParameters_;
};

#include "ShapeFactor.cpp"

#endif
