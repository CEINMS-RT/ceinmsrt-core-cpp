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

