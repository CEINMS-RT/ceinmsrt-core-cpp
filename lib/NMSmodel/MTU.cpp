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

#include "Curve.h"
#include <iostream>
using std::cout;
using std::endl;
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include <MTU.h>


#ifndef RADTODEG
#define RADTODEG
inline double radians ( double d )
{

	return d * M_PI / 180;
}

inline double degrees ( double r )
{

	return r * 180 / M_PI;
}
#endif

template<typename Activation, typename Tendon, typename CurveM>
MTU<Activation, Tendon, CurveM>::MTU()
	: id_ ( "" ), c1_ ( 0. ), c2_ ( 0. ), shapeFactor_ ( 0. ), activation_ ( 0. ),
		optimalFibreLength_ ( 0. ), pennationAngle_ ( 0. ), tendonSlackLength_ ( 0. ),
		percentageChange_ ( 0. ), fibreLength_ ( 0. ),
		fibreVelocity_ ( 0. ), damping_ ( 0. ), maxIsometricForce_ ( 0. ), time_ ( .0 ), timeScale_ ( 0. ),
		strengthCoefficient_(0.), muscleForce_(0.), pennationAngleInstantanous_(0.), filter_(false),
		normFibreLength_{0}, normFibreVelocity_{0}{} // Causes problems in the first time step if not initialized


template<typename Activation, typename Tendon, typename CurveM>
MTU<Activation, Tendon, CurveM>::MTU ( std::string id )
	: id_ ( id ), c1_ ( 0. ), c2_ ( 0. ), shapeFactor_ ( 0. ), activation_ ( 0. ),
		optimalFibreLength_ ( 0. ), pennationAngle_ ( 0. ), tendonSlackLength_ ( 0. ),
		percentageChange_ ( 0. ), fibreLength_ ( 0. ),
		fibreVelocity_ ( 0. ), damping_ ( 0. ), maxIsometricForce_ ( 0. ), time_ ( .0 ), timeScale_ ( 0. ),
		strengthCoefficient_(0.), muscleForce_(0.), tendonDynamic_(id), pennationAngleInstantanous_(0.), filter_(false),
		normFibreLength_{0}, normFibreVelocity_{0}{} // Causes problems in the first time step if not initialized



template<typename Activation, typename Tendon, typename CurveM>
MTU<Activation, Tendon, CurveM>::MTU ( const MTU<Activation, Tendon, CurveM>& orig )
{

	id_ = orig.id_;
	activationDynamic_ = orig.activationDynamic_;
	activation_ = orig.activation_;
	c1_ = orig.c1_;
	c2_ = orig.c2_;
	shapeFactor_ = orig.shapeFactor_;

	tendonDynamic_ = orig.tendonDynamic_;
	fibreVelocity_ = orig.fibreVelocity_;
	normFibreLength_ = orig.normFibreLength_;
	normFibreVelocity_ = orig.normFibreVelocity_;
	fibreLength_ = orig.fibreLength_;
	fibreLengthTrace_ = orig.fibreLengthTrace_;
	muscleForce_ = orig.muscleForce_;

	optimalFibreLength_ = orig.optimalFibreLength_;
	pennationAngle_ = orig.pennationAngle_;
	tendonSlackLength_ = orig.tendonSlackLength_;
	percentageChange_ = orig.percentageChange_;
	damping_ = orig.damping_;
	maxIsometricForce_ = orig.maxIsometricForce_;
	strengthCoefficient_ = orig.strengthCoefficient_;
	forceVelocityCurve_ = orig.forceVelocityCurve_;
	activeForceLengthCurve_ = orig.activeForceLengthCurve_;
	passiveForceLengthCurve_ = orig.passiveForceLengthCurve_;

	timeScale_ = orig.timeScale_;
	time_ = orig.time_;
	pennationAngleInstantanous_ = orig.pennationAngle_;
	filter_ = orig.filter_;
	filterFibVel_ = orig.filterFibVel_;
}



template<typename Activation, typename Tendon, typename CurveM>
MTU<Activation, Tendon, CurveM>& MTU<Activation, Tendon, CurveM>::operator= ( const MTU<Activation, Tendon, CurveM>& orig )
{

	id_ = orig.id_;
	activationDynamic_ = orig.activationDynamic_;
	activation_ = orig.activation_;
	c1_ = orig.c1_;
	c2_ = orig.c2_;
	shapeFactor_ = orig.shapeFactor_;

	tendonDynamic_ = orig.tendonDynamic_;
	fibreVelocity_ = orig.fibreVelocity_;
	normFibreVelocity_ = orig.normFibreVelocity_;
	fibreLength_ = orig.fibreLength_;
	fibreLengthTrace_ = orig.fibreLengthTrace_;
	muscleForce_ = orig.muscleForce_;

	optimalFibreLength_ = orig.optimalFibreLength_;
	pennationAngle_ = orig.pennationAngle_;
	tendonSlackLength_ = orig.tendonSlackLength_;
	percentageChange_ = orig.percentageChange_;
	damping_ = orig.damping_;
	maxIsometricForce_ = orig.maxIsometricForce_;
	strengthCoefficient_ = orig.strengthCoefficient_;
	forceVelocityCurve_ = orig.forceVelocityCurve_;
	activeForceLengthCurve_ = orig.activeForceLengthCurve_;
	passiveForceLengthCurve_ = orig.passiveForceLengthCurve_;

	timeScale_ = orig.timeScale_;
	time_ = orig.time_;

	pennationAngleInstantanous_ = orig.pennationAngle_;
	filter_ = orig.filter_;
	filterFibVel_ = orig.filterFibVel_;
	return *this;
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setParametersToComputeForces ( double optimalFibreLength,
		double pennationAngle,
		double tendonSlackLength,
		double percentageChange,
		double damping,
		double maxIsometricForce,
		double strengthCoefficient )
{
	optimalFibreLength_ = optimalFibreLength;
	pennationAngle_ = pennationAngle;
	tendonSlackLength_ = tendonSlackLength;
	percentageChange_ = percentageChange;
	damping_ = damping;
	maxIsometricForce_ = maxIsometricForce;
	strengthCoefficient_ = strengthCoefficient;

	tendonDynamic_.setParametersToComputeForces ( optimalFibreLength,
			pennationAngle,
			tendonSlackLength,
			percentageChange,
			damping,
			maxIsometricForce,
			strengthCoefficient );
	resetState();

	std::vector<double> a, b, pastData; 
	//a.push_back(1);
	/*a.push_back(-1.77863178);
	a.push_back(0.80080265);
	b.push_back(0.00554272);
	b.push_back(0.01108543);
	b.push_back(0.00554272); */// order: 2 fc: 400 Fs:10

	/*a.push_back(-1.6556);
	a.push_back(0.7068);
	b.push_back(0.0128); 
		b.push_back(0.0256);
	b.push_back(0.0128); // order: 2 fc: 256 Fs:10*/

	a.push_back(-1.3229);
	a.push_back(0.5000);
	b.push_back(0.0443);
	b.push_back(0.0886);
	b.push_back(0.0443); // order: 2 fc: 128 Fs:10

	pastData.push_back(0);
	pastData.push_back(0);
	pastData.push_back(0);
	filterFibVel_.init(a, b, pastData);
	filter_ = false;
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setParametersToComputeActivation ( double c1, double c2, double shapeFactor )
{

	c1_ = c1;
	c2_ = c2;
	shapeFactor_ = shapeFactor;
	activationDynamic_.setFilterParameters ( c1_, c2_ );
	activationDynamic_.setShapeFactor ( shapeFactor_ );
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::resetState()
{

	activationDynamic_.resetState();
	tendonDynamic_.resetState();
	fibreLengthTrace_.reset();
	filterFibVel_.reset();	// Filter has to be reset, otherwise non-zero outputs will be seen even after reseting the traces.
	activation_ = .0;
	muscleForce_ = 0;
	fibreLength_ = 0;
	normFibreLength_ = 0;
	normFibreVelocity_ = 0;
	fibreVelocity_ = 0;
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setTime ( const double& time )
{

	timeScale_ = time - time_;
	time_ = time;
	tendonDynamic_.setTime ( time );
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setEmg ( double emg )
{

	activationDynamic_.setEmg ( emg );
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setMuscleTendonLength ( double muscleTendonLength )
{
	tendonDynamic_.setMuscleTendonLength ( muscleTendonLength );
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateActivation()
{
	activationDynamic_.updateActivation();
	activation_ = activationDynamic_.getActivation();
	tendonDynamic_.setActivation ( activation_ );
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateFibreLengthAndVelocity()
{
	updateFibreLength();
	updateFibreVelocity();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateFibreLength()
{

	tendonDynamic_.updateFibreLength();
	fibreLength_ = tendonDynamic_.getFibreLength();
	double tendonLength = tendonDynamic_.getTendonLength();
	tendonStrain_ = ((tendonLength - tendonSlackLength_) / tendonSlackLength_);
	fibreLengthTrace_.addPointOnly ( time_, fibreLength_ );
	//std::cout << "fibreLength_ " << time_ << " ; " << fibreLength_ << std::endl << std::flush;
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateFibreVelocity()
{
	//std::cout << "updateFibreVelocity" << std::endl << std::flush;
	fibreLengthTrace_.refresh();
	fibreVelocity_ = fibreLengthTrace_.getFirstDerivative ( time_ );
	if (filter_)
		fibreVelocity_ = filterFibVel_.filter(fibreVelocity_);
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateFibreLength_OFFLINEPREP()
{
	updateFibreLength();
}

template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateFibreLengthAndVelocity_HYBRID()
{
	
	//updateFibreLength();
	fibreLengthTrace_.removeLastPointNoUpdate();
	tendonDynamic_.updateFibreLength();
	double tendonLength = tendonDynamic_.getTendonLength();
	tendonStrain_ = ((tendonLength - tendonSlackLength_) / tendonSlackLength_);
	fibreLength_ = tendonDynamic_.getFibreLength();
	fibreLengthTrace_.addPointOnly ( time_, fibreLength_ );

	//std::cout << "fibreLength_ " << time_ << " ; " << fibreLength_ << std::endl << std::flush;
	fibreLengthTrace_.refresh();
	updateFibreVelocity();
}



template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateMuscleForce()
{

	double optimalFiberLengthAtT = optimalFibreLength_ * ( percentageChange_ * ( 1.0 - activation_ ) + 1 );
////:TODO: strong review with the code... lot of check for closeness to 0
	double normFiberLength = fibreLength_ / optimalFibreLength_;
	double normFiberLengthAtT = fibreLength_ / optimalFiberLengthAtT;// Updated from CEINMS 0.9
	//:TODO: THIS IS WRONG! timeScale_?  0.1 should be timeScale_
	// double normFiberVelocity = timescale_ *fiberVelocity_ / optimalFiberLengthAtT;
	//double normFiberVelocity = 0.02 * fibreVelocity_ * 0.1 / optimalFiberLengthAtT;
	double fiberVel = fibreVelocity_ / optimalFibreLength_; // Updated from CEINMS 0.9
	if (fiberVel > 10) // 10 =  maxContractionVelocity
		fiberVel = 10;
	if (fiberVel < -10)
		fiberVel = -10;
	double normFiberVelocity = 0.1 * fiberVel;
	 //filter fiber velocity if signal is noisy
	double fv = forceVelocityCurve_.getValue ( normFiberVelocity );
	double fp = passiveForceLengthCurve_.getValue ( normFiberLength ); //normFiberLengthAtT to be updated
	double fa = activeForceLengthCurve_.getValue(normFiberLengthAtT);// Updated from CEINMS 0.9
    double pennationAngleAtT = computePennationAngle(optimalFibreLength_);
	//double pennationAngleAtT = computePennationAngle ( optimalFiberLengthAtT );
	
	
	pennationAngleInstantanous_ = pennationAngleAtT;
	normFibreLength_ = normFiberLengthAtT;
	normFibreVelocity_ = normFiberVelocity;

	muscleForceActive_ = maxIsometricForce_ * strengthCoefficient_ *
		(fa * fv * activation_) *
		cos(radians(pennationAngleAtT));

	muscleForcePassive_ = maxIsometricForce_ * strengthCoefficient_ *
		(fp + damping_ * normFiberVelocity) *
		cos(radians(pennationAngleAtT));

	muscleForce_ = muscleForcePassive_ + muscleForceActive_;

	if (muscleForce_ < 0)
		muscleForce_ = 0;
	/*if (id_ == "lat_gas_l")
	{
		std::cout << "time_ " << std::setprecision(15) << time_ << std::endl << std::flush;
		std::cout << "fibreLength_ " << std::setprecision(15) << fibreLength_ << std::endl;
		std::cout << "fibreVelocity_ " << std::setprecision(15) << fibreVelocity_ << std::endl;
		std::cout << "muscleForce_ " << std::setprecision(15) << muscleForce_ << std::endl;
		std::cout << "fv " << std::setprecision(15) << fv << std::endl;
		std::cout << "fp " << std::setprecision(15) << fv << std::endl;
		std::cout << "fa " << std::setprecision(15) << fv << std::endl;
		std::cout << std::endl;
	}*/
	/*
	    COUT;
	std::cout << "muscleForce_ " << muscleForce_ << std::endl;
	std::cout << "activation_ " << activation_ << std::endl;
	std::cout << "cos(radians(pennationAngleAtT)) " << cos(radians(pennationAngleAtT)) << std::endl;
	std::cout << "normFiberVelocity " << normFiberVelocity << std::endl;
	std::cout << "normFiberLength " << normFiberLength << std::endl;
	std::cout << "optimalFiberLengthAtT " << optimalFiberLengthAtT << std::endl;
	std::cout << "fibreVelocity_ " << fibreVelocity_ << std::endl;
	std::cout << "fibreLength_ " << fibreLength_ << std::endl;*/
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::pushState()
{

	activationDynamic_.pushState();
	tendonDynamic_.pushState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::resetFibreLengthTrace()
{

	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::updateFibreLengthTrace()
{

	fibreLengthTrace_.refresh();
	activationDynamic_.resetState();
	tendonDynamic_.resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setCurves ( const CurveM& activeForceLengthCurve,
		const CurveM& passiveForceLengthCurve,
		const CurveM& forceVelocityCurve )
{

	activeForceLengthCurve_  = activeForceLengthCurve;
	passiveForceLengthCurve_ = passiveForceLengthCurve;
	forceVelocityCurve_      = forceVelocityCurve;

	tendonDynamic_.setCurves ( activeForceLengthCurve, passiveForceLengthCurve, forceVelocityCurve );
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setActivation ( double activation )
{

	activation_ = activation;
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setShapeFactor ( double shapeFactor )
{

	shapeFactor_ = shapeFactor;
	activationDynamic_.setShapeFactor ( shapeFactor_ );
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setC1 ( double c1 )
{

	c1_ = c1;
	activationDynamic_.setFilterParameters ( c1_, c2_ );
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setC2 ( double c2 )
{

	c2_ = c2;
	activationDynamic_.setFilterParameters ( c1_, c2_ );
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setStrengthCoefficient ( double strengthCoefficient )
{

	strengthCoefficient_ = strengthCoefficient;
	tendonDynamic_.setStrengthCoefficient ( strengthCoefficient_ );
	resetState();
}

template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setOptimalFiberLengths ( double optimalFiberLength )
{
	optimalFibreLength_ = optimalFiberLength;
	tendonDynamic_.setOptimalFibreLength(optimalFiberLength);
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
void MTU<Activation, Tendon, CurveM>::setTendonSlackLength ( double tendonSlackLength )
{

	tendonSlackLength_ = tendonSlackLength;
	tendonDynamic_.setTendonSlackLength ( tendonSlackLength_ );
	resetState();
}


template<typename Activation, typename Tendon, typename CurveM>
double MTU<Activation, Tendon, CurveM>::getPenalty() const
{
	double penalty = 0.0;
	if (fabs(fibreLength_ / optimalFibreLength_ - 1.0) > 0.5) // changed from 0.5 
		penalty += fabs(fibreLength_ / optimalFibreLength_ - 1.0) * 100.;//like in the offline version //return 0.5;
	if (getTendonStrain() <= 0.0)
		penalty += abs(getTendonStrain()) * 100; //we really don't want negative tendon strains (which result in peaks in stiffness and force)
	
	return penalty;
}


template<typename Activation, typename Tendon, typename CurveM>
inline double MTU<Activation, Tendon, CurveM>::computePennationAngle ( double optimalFiberLength )
{

	double value = optimalFiberLength * sin ( radians ( pennationAngle_ ) ) / fibreLength_;
	
// 	COUT << optimalFiberLength << " ; " << value << std::endl << std::flush;


	if ( value <= 0.0 )
		pennationAngleInstantanous_ = 0.0;
	else if ( value >= 1.0 )
		pennationAngleInstantanous_ = 90.0;
	else
		pennationAngleInstantanous_ = degrees ( asin ( value ) );

	return pennationAngleInstantanous_;
}


template<typename Activation, typename Tendon, typename CurveM>
std::ostream& operator<< ( std::ostream& output, const MTU<Activation, Tendon, CurveM>& m )
{
	output << "Name: " << m.id_  << endl;
	output << "C1: " << m.c1_ << " C2: " << m.c2_ << endl;
	output << "Shape Factor: " << m.shapeFactor_ << endl;
	output << "activeForceLength" << endl << m.activeForceLengthCurve_ << endl;
	output << "passiveForceLength" << endl << m.passiveForceLengthCurve_ << endl;
	output << "forceVelocity" << endl << m.forceVelocityCurve_ << endl;
	output << "fibreLengthTrace" << endl << m.fibreLengthTrace_ << endl;
	output << "optimalFibreLength: " <<  m.optimalFibreLength_ << endl;
	output << "pennationAngle: " << m.pennationAngle_ << endl;
	output << "tendonSlackLength: " << m.tendonSlackLength_ << endl;
	output << "percentageChange: " << m.percentageChange_ << endl;
	output << "damping: " << m.damping_ << endl;
	output << "maxIsometricForce: " << m.maxIsometricForce_ << endl;
	output << "strengthCoefficient: " << m.strengthCoefficient_ << endl;
	output << "muscleTendonForce: " << m.muscleForce_ << endl;
	// :TODO: valli a mettere anche nel costruttore di copia

	return output;
}


