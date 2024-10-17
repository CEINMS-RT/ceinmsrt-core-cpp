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
#include "ExponentialActivation.h"
#include <math.h>

ExponentialActivation::ExponentialActivation():
beta1_(0.0), beta2_(0.0), alpha_(0.0), shapeFactor_(0.0),   
emg_(0.0), noOfEmg_(0), pastEmgT1_(0.0), pastEmgT2_(0.0), neuralActivation_(0.0),   
neuralActivationT1_(0.0), neuralActivationT2_(0.0), activation_(0.0), expShapeFactor_(0.0)

{
    
}

void ExponentialActivation::setFilterParameters (double c1, double c2) {

    beta1_ = c1 + c2;
    beta2_ = c1 * c2;
    alpha_ = 1 + beta1_ + beta2_;
}


void ExponentialActivation::setShapeFactor (double shapeFactor) {

    shapeFactor_ = shapeFactor;
    expShapeFactor_ = exp(shapeFactor_);
}

void ExponentialActivation::setEmg (double emg) {
    
    emg_ = emg;
}


void ExponentialActivation::updateActivation() {

    if ( noOfEmg_ == 2 ) {
        neuralActivationT1_ = (pastEmgT2_ + pastEmgT1_)/2;
        neuralActivationT2_ = neuralActivationT1_;
    } 
    
    neuralActivation_ = (alpha_*emg_) - (beta1_*neuralActivationT1_) - (beta2_*neuralActivationT2_);
    activation_ = ( exp(shapeFactor_*neuralActivation_) - 1 ) / (expShapeFactor_ - 1);
}


void ExponentialActivation::pushState() {

    neuralActivationT2_ = neuralActivationT1_;
    neuralActivationT1_ = neuralActivation_;
    pastEmgT2_ = pastEmgT1_;
    pastEmgT1_ = emg_;
    noOfEmg_++;
}


void ExponentialActivation::resetState() {

    emg_ = .0;
    activation_ = .0;
    neuralActivation_ = .0;
    neuralActivationT1_ = .0;
    neuralActivationT2_ = .0;
    pastEmgT1_ = .0;
    pastEmgT2_ = .0;
    noOfEmg_ = 0;
}


ExponentialActivation::~ExponentialActivation() { } 

