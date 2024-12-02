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

#ifndef ComputationMode_Default_h
#define ComputationMode_Default_h

#include <vector>
#include "TrialData.h"

/* 
 * You should use this computation mode when calibrating. It compute the model in the right way.
 * forceDataT1_ and activationDataT1_ are used to speed up the torque computation, saving the
 * force and the activation states at the previous calibration step. Works very well with
 * simulated annealing
 */

template<typename NMSmodelT>
class ComputationMode_Default {
  
public:
    ComputationMode_Default(NMSmodelT& subject);
    void setTrials(const std::vector<TrialData>& trials);
    ComputationMode_Default& operator=(const ComputationMode_Default& orig);
    void computeTorques(std::vector< std::vector< std::vector<double> > >& torques);
private:
    void getMusclesToUpdate();
    void initFiberLengthTraceCurves(unsigned trialIndex);
    NMSmodelT& subject_;
    std::vector< TrialData > trials_;
    std::vector<MuscleParameters> parametersT1_;
    std::vector< std::vector < std::vector<double> > > forceDataT1_; //1st trial, 2nd row, 3rd muscle
    std::vector< std::vector < std::vector<double> > > activationDataT1_;   //1st trial, 2nd row, 3rd muscle
    std::vector<unsigned> musclesToUpdate_;
};

#include "ComputationMode_Default.cpp"

#endif
