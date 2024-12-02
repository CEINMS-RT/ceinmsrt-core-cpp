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

#ifndef MTU_h
#define MTU_h

#include <string>
#include <iostream>

#include "Curve.h"
#include <CommonCEINMS.h>
#include <boost/timer/timer.hpp>
#include "Activation/ExponentialActivation.h"
#include "Activation/ExponentialActivationRT.h"
#include "Tendon/StiffTendon.h"
#include "Tendon/ElasticTendon.h"
#include "Tendon/ElasticTendon_BiSec.h"
#include <iomanip>
#include <Filter.h>

template<typename Activation, typename Tendon, typename CurveM>
class MTU;

template<typename Activation, typename Tendon, typename CurveM>
std::ostream& operator<<  (std::ostream& output, const MTU<Activation, Tendon, CurveM>& m);



template<typename Activation, typename Tendon, typename CurveM>
class MTU {
    
public:
    MTU();
    MTU(std::string id);
    MTU(const MTU& orig);
    MTU& operator=(const MTU& orig);
	virtual ~MTU() {};
    
    void setParametersToComputeForces(double optimalFiberLength,
                                      double pennationAngle,
                                      double tendonSlackLength,
                                      double percentageChange,
                                      double damping, 
                                      double maxIsometricForce, 
                                      double strengthCoefficient); 
                                   
    void setParametersToComputeActivation(double c1,
                                          double c2,
                                          double shapeFactor);
   
    std::string getMuscleName() const { return id_; }
    bool   compareMusclesId(const std::string& id) const { return id_ == id;}
    void   printName(std::ostream& output) { output << id_; }

    void   setTime(const double& time);
    void   setEmg(double emg);
    void   setMuscleTendonLength(double muscleTendonLength); 
    void   updateActivation();
    void   updateFibreLengthAndVelocity();  
    void   updateFibreLengthAndVelocity_HYBRID();
    void   updateFibreLength_OFFLINEPREP();
    void   updateMuscleForce();
    void   pushState();
	
	double getMuscleTendonLength() const {return tendonDynamic_.getMuscleTendonLength();};
    
    
 //   void   setEmgValueNoUpdate(double emg);
 //   void   setEmgValueNoNeuralFilterUpdate(double emg);
    double getEmg() const { return activationDynamic_.getEmg(); }
    double getPastEmg() const { return activationDynamic_.getPastEmg(); }
    double getNeuralActivation() const { return activationDynamic_.getNeuralActivation(); }
    void   setCurves(const CurveM& activeForceLengthCurve, const CurveM& passiveForceLengthCurve, const CurveM& forceVelocityCurve);
    void   setActivation(double activation); //recompute all the data that depend on activation
    double getActivation() const {return activation_;}
    void   setShapeFactor(double shapeFactor);
    double getShapeFactor() const {return shapeFactor_;}
    void   setC1(double c1);
    double getC1() const {return c1_;}
    void   setC2(double c2);
    double getC2() const {return c2_;}
    void   setStrengthCoefficient(double strengthCoefficient);
    void   setOptimalFiberLengths(double optimalFiberLength);
    double getStrengthCoefficient() const {return strengthCoefficient_;}
    void   setTendonSlackLength(double tendonSlackLength);
    double getTendonSlackLength() const {return tendonSlackLength_;};
    double getOptimalFiberLength() const { return optimalFibreLength_;}
    double getPennationAngleInst() const {return pennationAngleInstantanous_; }
    double getPennationAngle() const {return pennationAngle_; }
    double getTendonLength() const {return tendonDynamic_.getTendonLength();}
    double getTendonStrain() const { return tendonStrain_; }

    double getFiberLength() const {return fibreLength_;}
	double getNormFiberLength() const { return normFibreLength_; }
    double getFiberVelocity() const {return fibreVelocity_; }
    double getNormFiberVelocity() const {return normFibreVelocity_; } 
    double getMuscleForce() const {return muscleForce_;}
    double getMusclePassiveForce() const { return muscleForcePassive_; }
    double getMuscleActiveForce() const { return muscleForceActive_; }
    double getPercentageChange() const { return percentageChange_;}
    double getDamping() const {return damping_;}
    double getMaxIsometricForce() const { return maxIsometricForce_;}
    

  
    void   setMuscleForce(double muscleForce) {muscleForce_ = muscleForce;}
    void   updateFibreLengthTrace();
    void   resetFibreLengthTrace();
    double getPenalty() const;
    inline double computePennationAngle(double optimalFiberLengthAtT);
    
    friend std::ostream& operator<< <> (std::ostream& output, const MTU& m);
    
private:
    void updateFibreLength();
    void updateFibreVelocity();
    void updateLastFibreVelocityValue(double time);
    void resetState();
    
    std::string id_;
    
    // MUSCLE ACTIVATION: parameters and utility methods 
    Activation activationDynamic_;
   
    double activation_;    /**< \f$a(t)\f$ activation of the muscle */

    // Parameters to compute muscle activation
    double c1_;            /**< \f$C_1\f$ second-order filter coefficient */ 
    double c2_;            /**< \f$C_2\f$ second-order filter coefficient */
    double shapeFactor_;   /**< \f$A\f$ non-linear shape factor */
    
    // FIBER LENGTH, FIBER VELOCITY, and TENDON FORCE: 
    Tendon tendonDynamic_;
    double fibreVelocity_;       /**< \f$v^m\f$ */
    double normFibreVelocity_; 
    double fibreLength_;         /**< \f$l^m\f$ */  
	double normFibreLength_;       
    CurveM  fibreLengthTrace_;
    double muscleForce_;    /**< \f$F^{mt}\f$ */
    double muscleForceActive_;
    double muscleForcePassive_;
    double tendonStrain_;

    // parameters
    double optimalFibreLength_;  /**< \f$l_0^m\f$ */
    double pennationAngle_;      /**< \f$\phi\f$ */
    double pennationAngleInstantanous_;
    double tendonSlackLength_;   /**< \f$l^t\f$ */
    double percentageChange_;    /**< \f$\gamma\f$ */
    double damping_;             /**< \f$d^m\f$ */
    double maxIsometricForce_;   /**< \f$F_0^m\f$ */
    double strengthCoefficient_;  /**< \f$\delta\f$ */
    CurveM  forceVelocityCurve_;
    CurveM  activeForceLengthCurve_;
    CurveM  passiveForceLengthCurve_;
	FilterKin::Filter<double> filterFibVel_;
	bool filter_;

    double timeScale_;
    double time_;
};

template<>
inline void MTU<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, Curve<CurveMode::Offline> >::updateFibreLength()
{
	tendonDynamic_.updateFibreLength();
	fibreLength_ = tendonDynamic_.getFibreLength();
}


template<>
inline void MTU<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, Curve<CurveMode::Offline> >::updateFibreVelocity()
{
	fibreVelocity_ = fibreLengthTrace_.getFirstDerivative ( time_ );
}

template<>
inline void MTU<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, Curve<CurveMode::Offline> >::updateFibreLengthAndVelocity()
{
	updateFibreLength();
	updateFibreVelocity();
}

template<>
inline void MTU<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, Curve<CurveMode::Offline> >::updateFibreLength()
{
	tendonDynamic_.updateFibreLength();
	fibreLength_ = tendonDynamic_.getFibreLength();
}


template<>
inline void MTU<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, Curve<CurveMode::Offline> >::updateFibreVelocity()
{
	fibreVelocity_ = fibreLengthTrace_.getFirstDerivative ( time_ );
}

template<>
inline void MTU<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, Curve<CurveMode::Offline> >::updateFibreLengthAndVelocity()
{
	updateFibreLength();
	updateFibreVelocity();
}

#include "MTU.cpp"

#endif
