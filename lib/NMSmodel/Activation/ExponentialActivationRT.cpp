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

#include "ExponentialActivationRT.h"
#include <math.h>
#include <stdio.h>
#include <iostream>

ExponentialActivationRT::ExponentialActivationRT():shapeFactor_(0.0),
emg_(0.0), noOfEmg_(0), activation_(0.0), expShapeFactor_(0.0), emgPast_(0.0)

{
    
}

void ExponentialActivationRT::setFilterParameters(double c1, double c2)
{

}

void ExponentialActivationRT::setShapeFactor (double shapeFactor) {

    shapeFactor_ = shapeFactor;
    expShapeFactor_ = exp(shapeFactor_);
}

void ExponentialActivationRT::setEmg (double emg) {
	emgPast_ = emg_;
    emg_ = emg;
}


void ExponentialActivationRT::updateActivation() {
    activation_ = ( exp(shapeFactor_*emg_) - 1 ) / (expShapeFactor_ - 1);
}


void ExponentialActivationRT::pushState() {
    noOfEmg_++;
}


void ExponentialActivationRT::resetState() {

    emg_ = .0;
    activation_ = .0;
    noOfEmg_ = 0;
}


ExponentialActivationRT::~ExponentialActivationRT() { }

