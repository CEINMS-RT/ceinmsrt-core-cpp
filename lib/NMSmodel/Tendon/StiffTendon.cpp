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

#include <string>
#define _USE_MATH_DEFINES
#include <math.h>


#ifndef RADTODEG
#define RADTODEG
inline double radians (double d) {
return d * 3.1415926535897932384626433832795 / 180;
}

inline double degrees (double r) {
return r * 180/ 3.1415926535897932384626433832795;
}

#endif

template <typename CurveM>
StiffTendon<CurveM>::StiffTendon()
{

}

template <typename CurveM>
StiffTendon<CurveM>::StiffTendon(std::string id):
id_(id)

{

}

template <typename CurveM>
StiffTendon<CurveM>::StiffTendon (double optimalFibreLength, 
                          double pennationAngle, 
                          double tendonSlackLength, 
                          double percentageChange, 
                          double damping, 
                          double maxIsometricForce, 
                          double strengthCoefficient, 
                          const CurveM& activeForceLengthCurve, 
                          const CurveM& passiveForceLengthCurve, 
                          const CurveM& forceVelocityCurve):
optimalFibreLength_(optimalFibreLength),
pennationAngle_(pennationAngle),
tendonSlackLength_(tendonSlackLength),
percentageChange_(percentageChange)                         
{

}

template <typename CurveM>
void StiffTendon<CurveM>::setParametersToComputeForces(double optimalFibreLength,
                                               double pennationAngle,
                                               double tendonSlackLength,
                                               double percentageChange,
                                               double damping, 
                                               double maxIsometricForce, 
                                               double strengthCoefficient) {
    
    optimalFibreLength_ = optimalFibreLength;
    pennationAngle_     = pennationAngle;
    tendonSlackLength_  = tendonSlackLength;
    percentageChange_   = percentageChange;
    
}

template <typename CurveM>
void StiffTendon<CurveM>::setMuscleTendonLength(double muscleTendonLength) {

    muscleTendonLength_ = muscleTendonLength;
}

template <typename CurveM>
void StiffTendon<CurveM>::setActivation(double activation) {
    
    activation_ = activation;
}

template <typename CurveM>
void StiffTendon<CurveM>::updateFibreLength() {
 
    double optimalFibreLengthAtT = optimalFibreLength_ * (percentageChange_ * (1.0 - activation_) + 1 );
    double first = optimalFibreLengthAtT * sin( radians(pennationAngle_));
    double second = muscleTendonLength_ - tendonSlackLength_;
    fibreLength_ = sqrt(first*first + second*second);
}

template <typename CurveM>
void StiffTendon<CurveM>::resetState() {
    
    activation_ = 0;
    muscleTendonLength_ = 0;
    fibreLength_ = 0;

}

template <typename CurveM>
StiffTendon<CurveM>::~StiffTendon()
{
   
}

// template class StiffTendon<Curve<CurveMode::Online> >;
// template class StiffTendon<Curve<CurveMode::Offline> >;
