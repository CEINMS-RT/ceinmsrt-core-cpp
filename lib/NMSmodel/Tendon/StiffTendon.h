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

#ifndef StiffTendon_h
#define StiffTendon_h
#include "Curve.h"
#include <string>

template <typename CurveM>
class StiffTendon {
    
public:
    StiffTendon();
    StiffTendon(std::string id);
    StiffTendon( double optimalFibreLength,
                   double pennationAngle,
                   double tendonSlackLength,
                   double percentageChange,
                   double damping,
                   double maxIsometricForce, 
                   double strengthCoefficient,
                   const CurveM& activeForceLengthCurve,
                   const CurveM& passiveForceLengthCurve, 
                   const CurveM& forceVelocityCurve
                 );
    virtual ~StiffTendon();

    
    void setParametersToComputeForces(double optimalFibreLength,
                                      double pennationAngle,
                                      double tendonSlackLength,
                                      double percentageChange,
                                      double damping, 
                                      double maxIsometricForce, 
                                      double strengthCoefficient); 

    void setTime(const double& time) {} // it is useless in the stiff tendon, but I need to respect the interface
    void setMuscleTendonLength(double muscleTendonLength);
    void setActivation(double activation);
    void updateFibreLength();

    double getFibreLength() { return fibreLength_;}
    void setStrengthCoefficient(double strengthCoefficient) { };
    void setTendonSlackLength(double tendonSlackLength) { tendonSlackLength_ = tendonSlackLength; }
    void setOptimalFibreLength(double optimalFibreLength) { optimalFibreLength_ = optimalFibreLength; }
    void setCurves(const CurveM& activeForceLengthCurve, 
                   const CurveM& passiveForceLengthCurve, 
                   const CurveM& forceVelocityCurve) { };
                   
    void pushState() {};
    void resetState();
	double getTendonLength() const {return tendonSlackLength_;};
	double getMuscleTendonLength() const {return muscleTendonLength_;};

private:
    double optimalFibreLength_;
    double percentageChange_;
    double pennationAngle_;
    double tendonSlackLength_;
    
    double fibreLength_;
    double muscleTendonLength_;
    double activation_;
    
    std::string id_;
    
};

#include "StiffTendon.cpp"

#endif
