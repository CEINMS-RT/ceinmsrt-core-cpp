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
#ifndef ComputationMode_Fast_h
#define ComputationMode_Fast_h

#include <vector>
#include "TrialData.h"

/* 
 * You should use this computation mode when calibrating. It compute the model in the right way.
 * forceDataT1_ and activationDataT1_ are used to speed up the torque computation, saving the
 * force and the activation states at the previous calibration step. Works very well with
 * simulated annealing
 */

template<typename NMSmodelT>
class ComputationMode_Fast {
    
public:
    ComputationMode_Fast(NMSmodelT& subject);
    void setTrials(const std::vector<TrialData>& trials);
    ComputationMode_Fast& operator=(const ComputationMode_Fast& orig);
    void computeTorques(std::vector< std::vector< std::vector<double> > >& torques, std::vector< std::vector< double > >& penalties);
    void computePenalties(std::vector< std::vector< double > >& penalties);
private:
    void getMusclesToUpdate();
    void initFiberLengthTraceCurves(unsigned trialIndex);
    NMSmodelT& subject_;
    std::vector< TrialData > trials_;
    std::vector<MuscleParameters> parametersT1_;
    std::vector< std::vector < std::vector<double> > > forceDataT1_; //1st trial, 2nd row, 3rd muscle
    std::vector< std::vector < std::vector<double> > > normFiberVelDataT1_; //1st trial, 2nd row, 3rd muscle
    std::vector<unsigned> musclesToUpdate_;
    
};

#include "ComputationMode_Fast.cpp"

#endif
