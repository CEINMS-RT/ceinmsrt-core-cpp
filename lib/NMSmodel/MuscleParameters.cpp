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

#include "MuscleParameters.h"
#include "float.h"

MuscleParameters::MuscleParameters():
c1_(-DBL_MAX), c2_(-DBL_MAX), shapeFactor_(-DBL_MAX),
optimalFiberLength_(-DBL_MAX), pennationAngle_(-DBL_MAX),
tendonSlackLength_(-DBL_MAX), maxIsometricForce_(-DBL_MAX),
strengthCoefficient_(-DBL_MAX)
{

}

bool MuscleParameters::operator==(const MuscleParameters &other) const {

    return  c1_                  == other.c1_                   &&
            c2_                  == other.c2_                   &&
            shapeFactor_         == other .shapeFactor_         &&
            optimalFiberLength_  == other.optimalFiberLength_   &&
            pennationAngle_      == other.pennationAngle_       &&   
            tendonSlackLength_   == other.tendonSlackLength_    &&
            maxIsometricForce_   == other.maxIsometricForce_    &&
            strengthCoefficient_ == other.strengthCoefficient_ ;
    
}


MuscleParameters::~MuscleParameters()
{

}
