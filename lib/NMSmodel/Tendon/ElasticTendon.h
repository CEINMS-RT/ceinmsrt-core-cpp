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

#ifndef ElasticTendon_h
#define ElasticTendon_h
#include "Curve.h"
#include <string>

#include "StiffTendon.h"
#include "Odeint.h"
#include "StepperDopr5.h"
#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
//#include <boost/config/no_tr1/complex.hpp>
using std::vector;
#include "float.h"

#include "Curve.h"


// class for the initial fiber legth computation. #Revised#
template <typename CurveM>
class LDFM { 

public:
    LDFM()  {}  
    LDFM( double optimalFiberLength,
          double pennationAngle,
          double tendonSlackLength,
          double percentageChange,
          double damping,
          double maxIsometricForce, 
          double strengthCoefficient,
          const CurveM& activeForceLengthCurveM,
          const CurveM& passiveForceLengthCurveM,
          const CurveM& forceVelocityCurveM
        );   
    virtual ~LDFM() {}
    
    
    void   setInitConditions(double muscleTendonLength, double muscleTendonVelocity, double activation);
    double estimateFiberLength();
    double oldEstimation(double muscleTendonLength, double muscleTendonVelocity, double activation);

private:
    void   updateInitConditions();
    void computeMuscleTendonForce();
    double computePennationAngle() const;
 
    
    //** parameters from MTU
    double optimalFiberLength_;
    double pennationAngle_;
    double tendonSlackLength_;
    double percentageChange_;
    double damping_;
    double maxIsometricForce_; 
    double strengthCoefficient_;
    CurveM activeForceLengthCurveM_;
    CurveM passiveForceLengthCurveM_;
    CurveM forceVelocityCurveM_;
    CurveM tendonForceStrainCurveM_;
    //**
    
    
    //** initial condition parameters
    double activation_;
    double muscleTendonLength_;
    double muscleTendonVelocity_;
    //**
    
    
    //** variables used for the LDFM method
    double muscleTendonForce_;
    double fiberLength_; 
    double fiberVelocity_; 
    double fiberStiffness_;
    double tendonLength_; 
    double tendonVelocity_; 
    double tendonStiffness_;
    double optimalFiberLengthAtT_;
    //**
};


template <typename CurveM>
class ElasticTendon {
    
public:

    ElasticTendon();
    ElasticTendon(std::string id_);
    ElasticTendon(double optimalFiberLength,
                  double pennationAngle,
                  double tendonSlackLength,
                  double percentageChange,
                  double damping,
                  double maxIsometricForce, 
                  double strengthCoefficient,
                  const CurveM& activeForceLengthCurveM,
                  const CurveM& passiveForceLengthCurveM, 
                  const CurveM& forceVelocityCurveM
                 );
    virtual ~ElasticTendon();
	
	inline void setTime(const double& time)
	{
		lmtTimeT1_ = lmtTime_;
		time_ = time;
		lmtTime_ = time;
		timescale_ = lmtTime_ - lmtTimeT1_;
	} 
	inline void setActivation(double activation)
	{
		activationT1_  = activation_;
		activation_ = activation;
	}
	inline void pushState()
	{
		fiberLengthT1_ = fiberLength_;
	}
	
	inline void updateFibreLength()
	{
		fiberLengthT1_ = fiberLength_;
		
		if(nLmt_ < 3)
			fiberLength_ = getInitialFiberLength(time_);
		else
			fiberLength_ = getFiberLengthRKF(); 
		
		++nLmt_;	
	}
	
	void setMuscleTendonLength(double muscleTendonLength);
	
	inline double getFibreLength() const { return fiberLength_;}
	double getMuscleTendonLength() const {return muscleTendonLength_;};
	
    ElasticTendon(const ElasticTendon<CurveM>& orig);
    ElasticTendon<CurveM>& operator=(const ElasticTendon<CurveM>& orig);
    void operator() (double x, const std::vector<double> &y, std::vector<double> &dydx);
    void setParametersToComputeForces(double optimalFiberLength,
                                      double pennationAngle,
                                      double tendonSlackLength,
                                      double percentageChange,
                                      double damping, 
                                      double maxIsometricForce, 
                                      double strengthCoefficient); 

    void setMuscleTendonLength(double muscleTendonLength, double activation, double time);
    void updateFiberLengthUsingNewActivation(double activation, double time);
    double getFiberLength() const { return fiberLength_;}
    void setStrengthCoefficient(double strengthCoefficient);
    void setTendonSlackLength(double tendonSlackLength);
    void setOptimalFibreLength(double optimalFibreLength) {
    	optimalFiberLength_ = optimalFibreLength;
    	resetState();
    }
    void setCurves(const CurveM& activeForceLengthCurveM, 
                   const CurveM& passiveForceLengthCurveM, 
                   const CurveM& forceVelocityCurveM);
    void resetState();
	double getTendonLength() const {return tendonLength_;};
    
private:
    double getInitialFiberLength(double time);
    double computeFiberVelocityAtT(double muscleTendonLengthAtT, double activationAtT, double muscleTendonVelocityAtT, double fiberLength, double lmtTimescale);
    double computeNormFiberVelocity(double activation, double activeForce, double velocityDependentForce); 
    
    double getFiberLengthRKF();
    
    //** Parameters from the MTU 
    double optimalFiberLength_;
    double pennationAngle_;
    double tendonSlackLength_;
    double percentageChange_;
    double damping_;
    double maxIsometricForce_; 
    double strengthCoefficient_;
	
    CurveM activeForceLengthCurveM_;
    CurveM passiveForceLengthCurveM_;
    CurveM forceVelocityCurveM_;
    CurveM tendonForceStrainCurveM_;
    CurveM muscleTendonLengthTrace_;
    //**
    
	double time_;
    double testTime_;
//    CurveM muscleTendonLengthTrace_; //verificare se questa serve in altre parti del tendine elastico
    double muscleTendonLength_;     //questa va passato dalla classe muscle -> vedere se si puo' evitare di farlo membre
    double muscleTendonLengthT1_;
    double muscleTendonVelocity_;   //questo viene derivato dall lunghezza mt -> come sopra
    double muscleTendonVelocityT1_;
    double fiberLength_;            //calcolato nella classe
    double fiberLengthT1_;          //valore della fiberLength al passo precedente
    double fiberVelocity_;
    double muscleTendonForce_;
    double activation_;             // da passare in qualche modo alla classe, potrebbe servire anche l'attivazione passata
    double activationT1_;
    double timescale_;          //da sistemare il valore hardcoded
    double lmtTime_;                   // da passare alla classe, forse si puo' evitare di utilizzare una variabile membro
    double lmtTimeT1_;
    unsigned nLmt_;
    double tendonLength_;
    std::string id_;
};

#include "ElasticTendon.cpp"

#endif

