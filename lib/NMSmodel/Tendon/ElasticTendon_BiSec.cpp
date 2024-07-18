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
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
using std::vector;
#include "float.h"
#include <iostream>
#include <fstream>
using std::cout;
using std::endl;
#include "ElasticTendon_BiSec.h"
//#define DEEP_DEBUG
// #define DEBUG

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

template <typename T>
T sign(const T& a, const T& b) {

	return sgn(b)*fabs(a);
}


#ifndef RADTODEG
#define RADTODEG
inline double radians(double d) {
	return d * 3.1415926535897932384626433832795 / 180;
}

inline double degrees(double r) {
	return r * 180 / 3.1415926535897932384626433832795;
}
#endif

#ifndef PENANGLE
#define PENANGLE
class PennationAngle {

public:
	static double compute(double fibreLength, double optimalFibreLength, double pennationAngle) {
		double value = optimalFibreLength * sin(radians(pennationAngle)) / fibreLength;
		if (value <= 0.0)
			return (0.0);
		else if (value >= 1.0)
			return (90.0);
		return (degrees(asin(value)));
	}
};
#endif


#ifndef TENDSPLINEPOINTS
#define TENDSPLINEPOINTS
struct TendonSplinePoints {

	static void getX(vector<double>& x) {

		x.clear();
		x.push_back(-10);
		x.push_back(-0.002);
		x.push_back(-0.001);
		x.push_back(0);
		x.push_back(0.00131);
		x.push_back(0.00281);
		x.push_back(0.00431);
		x.push_back(0.00581);
		x.push_back(0.00731);
		x.push_back(0.00881);
		x.push_back(0.0103);
		x.push_back(0.0118);
		x.push_back(0.0123);
		x.push_back(9.2);
		x.push_back(9.201);
		x.push_back(9.202);
		x.push_back(20);
	}

	static void getY(vector<double>& y) {

		y.clear();
		y.push_back(0);
		y.push_back(0);
		y.push_back(0);
		y.push_back(0);
		y.push_back(0.0108);
		y.push_back(0.0257);
		y.push_back(0.0435);
		y.push_back(0.0652);
		y.push_back(0.0915);
		y.push_back(0.123);
		y.push_back(0.161);
		y.push_back(0.208);
		y.push_back(0.227);
		y.push_back(345);
		y.push_back(345);
		y.push_back(345);
		y.push_back(345);
	}

};
#endif

template <typename CurveM>
ElasticTendon_BiSec<CurveM>::ElasticTendon_BiSec() :
	optimalFibreLength_(.0),
	pennationAngle_(.0),
	tendonSlackLength_(.0),
	percentageChange_(.0),
	damping_(.0),
	maxIsometricForce_(.0),
	strengthCoefficient_(.0),
	muscleTendonLength_(0.0),
	tendonLength_(0.0), //added to compute strain
	fibreLength_(0.0), // changed to optimalFibreLength
	fibreLengthT1_(0.0),
	activation_(0.0),
	timeScale_(0.005),
	tolerance_(.0000001),
	time_(0.0),
	id_("")
{
	vector<double> x, y;
	TendonSplinePoints::getX(x);
	TendonSplinePoints::getY(y);

	tendonForceStrainCurve_.resetPointsWith(x, y);
	/*	//I want to print the values of tendon force for strains from 0 to 0.05
		int nElements = 1000;// This could be added to the simulatedAnnealing.xml file

		double eps_0 = 0.0;
		double eps_end = 0.05;
		double eps_delta = (eps_end - eps_0) / (double(nElements) - 1);
		std::ofstream myFt;
		myFt.open("cfg\\OutputCalibration\\tendonForceFromSpline1000.out");

		std::ofstream myKt;
		myKt.open("cfg\\OutputCalibration\\tendonStiffnessFromSpline1000.out");

		for (int i = 0; i < nElements; i++) {
			myFt << tendonForceStrainCurve_.getValue(eps_0 + i * eps_delta) << "\n";
			myKt << tendonForceStrainCurve_.getFirstDerivative(eps_0 + i * eps_delta) << "\n";
		}
		myFt.close();
		myKt.close();
		*/
}

template <typename CurveM>
ElasticTendon_BiSec<CurveM>::ElasticTendon_BiSec(std::string id) :
	optimalFibreLength_(.0),
	pennationAngle_(.0),
	tendonSlackLength_(.0),
	percentageChange_(.0),
	damping_(.0),
	maxIsometricForce_(.0),
	strengthCoefficient_(.0),
	muscleTendonLength_(0.0),
	tendonLength_(0.0),//added to compute strain
	fibreLength_(0.0),
	fibreLengthT1_(0.0),
	activation_(0.0),
	timeScale_(0.005),
	tolerance_(.0000001),
	time_(0.0),
	id_(id) {

	vector<double> x, y;
	TendonSplinePoints::getX(x);
	TendonSplinePoints::getY(y);

	tendonForceStrainCurve_.resetPointsWith(x, y);
}
// the times i run the code it didn't enter this constructor
template <typename CurveM>
ElasticTendon_BiSec<CurveM>::ElasticTendon_BiSec(double optimalFibreLength,
	double pennationAngle,
	double tendonSlackLength,
	double percentageChange,
	double damping,
	double maxIsometricForce,
	double strengthCoefficient,
	const CurveM& activeForceLengthCurve,
	const CurveM& passiveForceLengthCurve,
	const CurveM& forceVelocityCurve) :

	optimalFibreLength_(optimalFibreLength),
	pennationAngle_(pennationAngle),
	tendonSlackLength_(tendonSlackLength),
	percentageChange_(percentageChange),
	damping_(damping),
	maxIsometricForce_(maxIsometricForce),
	strengthCoefficient_(strengthCoefficient),
	activeForceLengthCurve_(activeForceLengthCurve),
	passiveForceLengthCurve_(passiveForceLengthCurve),
	forceVelocityCurve_(forceVelocityCurve),
	muscleTendonLength_(0.0),
	tendonLength_(tendonSlackLength),//added to compute strain
	fibreLength_(optimalFibreLength_),
	fibreLengthT1_(optimalFibreLength_),
	activation_(0.0),
	timeScale_(0.005),
	tolerance_(.0000001),
	//time_(0.0),
	id_("")
{
	vector<double> x, y;
	TendonSplinePoints::getX(x);
	TendonSplinePoints::getY(y);

	tendonForceStrainCurve_.resetPointsWith(x, y);
}

template <typename CurveM>
ElasticTendon_BiSec<CurveM>::ElasticTendon_BiSec(const ElasticTendon_BiSec<CurveM>& orig) {

	cout << "ElasticTendon_BiSec copy constructor. EXIT\n";
	exit(EXIT_FAILURE);
}

template <typename CurveM>
ElasticTendon_BiSec<CurveM>& ElasticTendon_BiSec<CurveM>::operator= (const ElasticTendon_BiSec<CurveM>& orig) {

	optimalFibreLength_ = orig.optimalFibreLength_;
	pennationAngle_ = orig.pennationAngle_;
	tendonSlackLength_ = orig.tendonSlackLength_;
	percentageChange_ = orig.percentageChange_;
	damping_ = orig.damping_;
	maxIsometricForce_ = orig.maxIsometricForce_;
	strengthCoefficient_ = orig.strengthCoefficient_;
	activeForceLengthCurve_ = orig.activeForceLengthCurve_;
	passiveForceLengthCurve_ = orig.passiveForceLengthCurve_;
	forceVelocityCurve_ = orig.forceVelocityCurve_;
	tendonForceStrainCurve_ = orig.tendonForceStrainCurve_;

	muscleTendonLength_ = orig.muscleTendonLength_;
	tendonLength_ = orig.tendonSlackLength_;//added for tendon strain
	fibreLength_ = orig.fibreLength_;
	fibreLengthT1_ = orig.fibreLengthT1_;
	activation_ = orig.activation_;
	id_ = orig.id_;
	timeScale_ = orig.timeScale_;
	tolerance_ = orig.tolerance_;
	time_ = orig.time_;
	return *this;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setParametersToComputeForces(double optimalFiberLength,
	double pennationAngle,
	double tendonSlackLength,
	double percentageChange,
	double damping,
	double maxIsometricForce,
	double strengthCoefficient) {

	optimalFibreLength_ = optimalFiberLength;
	pennationAngle_ = pennationAngle;
	tendonSlackLength_ = tendonSlackLength;
	percentageChange_ = percentageChange;
	damping_ = damping;
	maxIsometricForce_ = maxIsometricForce;
	strengthCoefficient_ = strengthCoefficient;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setTime(const double& time) {
	timeScale_ = time - time_;
	time_ = time;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setMuscleTendonLength(double muscleTendonLength) {

	muscleTendonLength_ = muscleTendonLength;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setActivation(double activation) {

	activation_ = activation;
}


template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::updateFibreLength() {

	//const double tol = .001;
	//const double tol = .000001; //default value of offline
	const unsigned nIter = 100;
	tendonPenalty_ = .0;
	fibreLength_ = estimateFiberLengthBiSec(tolerance_, nIter);
	double pennationAngleAtT = PennationAngle::compute(fibreLength_, optimalFibreLength_, pennationAngle_);//added the update of tendonLength_
	tendonLength_ = muscleTendonLength_ - fibreLength_ * cos(radians(pennationAngleAtT));
	//	if (tendonLength_ < tendonSlackLength_)
	//		tendonLength_ = tendonSlackLength_;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::pushState() {

	fibreLengthT1_ = fibreLength_;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setCurves(const CurveM& activeForceLengthCurve,
	const CurveM& passiveForceLengthCurve,
	const CurveM& forceVelocityCurve) {

	activeForceLengthCurve_ = activeForceLengthCurve;
	passiveForceLengthCurve_ = passiveForceLengthCurve;
	forceVelocityCurve_ = forceVelocityCurve;
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setStrengthCoefficient(double strengthCoefficient) {

	strengthCoefficient_ = strengthCoefficient;
	resetState();
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::setTendonSlackLength(double tendonSlackLength) {

	tendonSlackLength_ = tendonSlackLength;
	resetState();
}

template <typename CurveM>
void ElasticTendon_BiSec<CurveM>::resetState() {

	muscleTendonLength_ = 0.0;
	fibreLength_ = optimalFibreLength_;
	fibreLengthT1_ = optimalFibreLength_;
	activation_ = 0.0;
	time_ = 0.0;
	timeScale_ = 0.005;

}

template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::computeNormalizedTendonStiffness(double tendonStrain) {
	return tendonForceStrainCurve_.getFirstDerivative(tendonStrain);
}

template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::estimateFiberLengthBiSec(double tol, unsigned maxIterations) {

	//   cout << "------------------\n";
	//   cout << "Fibre Length for " << id_ << endl; 

#ifdef DEEP_DEBUG
	cout << "Start DD\n";
	const unsigned nSteps = 100;
	double incr = 2.0*optimalFibreLength_ / nSteps;
	double fl = 0;//optimalFibreLength_*0.5;
	cout << "tendon Force\n";
	for (unsigned s = 0; s < nSteps; ++s) {
		fl += incr;
		//cout << fl << " " << evaluateForceError(fl) << endl;
		cout << fl << " " << computeTendonForce(fl) << endl;
	}
	cout << "muscle Force\n";
	fl = 0;
	for (unsigned s = 0; s < nSteps; ++s) {
		fl += incr;
		//cout << fl << " " << evaluateForceError(fl) << endl;
		cout << fl << " " << computeMuscleForce(fl) << endl;
	}
	cout << "End DD\n";
#endif

	bool runCondition = true;
	unsigned nIter = 0;

	double optimalFibreLengthAtT = optimalFibreLength_ * (percentageChange_ *
		(1.0 - activation_) + 1);

	double minFibreLength = 0.2*optimalFibreLength_;
	//double maxFibreLength = 1.8*optimalFibreLength_;
	double maxFibreLength = 2 * optimalFibreLength_; // value of ceinms offline
	double currentFibreLength = optimalFibreLength_;
#ifdef DEBUG
	cout << "Error @ minFibreLength " << evaluateForceError(minFibreLength) << endl;
	cout << "Error @ maxFibreLength " << evaluateForceError(maxFibreLength) << endl;
#endif

	try {
		currentFibreLength = wdbSolve(*this, minFibreLength, maxFibreLength, tol);
		//     currentFibreLength = rtSafe(*this, minFibreLength, maxFibreLength, tol);
	}
	catch (...) {

		//     cout << "Exception: cannot solve " << id_ << " setting currentFibreLength=optimalFibreLength\nSwitching to stiff tendon\n";
		currentFibreLength = getFibreLengthStiff();
		tendonPenalty_ += 100;
	}

	return currentFibreLength;

}

template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::operator()(double fl) {

	return evaluateForceError(fl);
}

template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::evaluateForceError(double fiberLength) {

	double tendonForce = computeTendonForce(fiberLength);
	double muscleForce = computeMuscleForce(fiberLength);
	//   cout << "tendonForce " << tendonForce << endl;
	//   cout << "muscleForce " << muscleForce << endl;
	return (tendonForce - muscleForce);
}

template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::computeTendonForce(double fibreLength) {
	//double optimalFiberLengthAtT = optimalFibreLength_ * (percentageChange_ *
	//	(1.0 - activation_) + 1);
	double pennationAngleAtT = PennationAngle::compute(fibreLength, optimalFibreLength_, pennationAngle_);
	double tendonLength = muscleTendonLength_ - fibreLength * cos(radians(pennationAngleAtT));
	//tendonLength_ = muscleTendonLength_ - fibreLength * cos(radians(pennationAngleAtT));
	double tendonStrain = (tendonLength - tendonSlackLength_) / tendonSlackLength_;
	double tendonForce = strengthCoefficient_ * maxIsometricForce_*
		tendonForceStrainCurve_.getValue(tendonStrain);

	return tendonForce;
}

template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::computeMuscleForce(double fibreLength) {

	double optimalFiberLengthAtT = optimalFibreLength_ * (percentageChange_ *
		(1.0 - activation_) + 1);

	double normFiberLengthAt = fibreLength / optimalFiberLengthAtT;
	double normFiberLength = fibreLength / optimalFibreLength_;

	double normFiberVelocity = (fibreLength - fibreLengthT1_) / (optimalFibreLength_ *  timeScale_); //added the time step

	if (normFiberVelocity > 10)
		normFiberVelocity = 10;
	if (normFiberVelocity < -10)
		normFiberVelocity = -10;
	normFiberVelocity /= 10;
	double fv = forceVelocityCurve_.getValue(normFiberVelocity);
	double fp = passiveForceLengthCurve_.getValue(normFiberLength);
	double fa = activeForceLengthCurve_.getValue(normFiberLengthAt);
	double pennationAngleAtT = PennationAngle::compute(fibreLength, optimalFibreLength_, pennationAngle_);
	
	double muscleForce = maxIsometricForce_ * strengthCoefficient_ *
		(fa * fv * activation_ + fp + damping_ * normFiberVelocity)*
		cos(radians(pennationAngleAtT));

	return muscleForce;
}

// the offline version has this function 
template <typename CurveM>
double ElasticTendon_BiSec<CurveM>::getFibreLengthStiff() const {

	double first = optimalFibreLength_ * sin(radians(pennationAngle_));
	double second = muscleTendonLength_ - tendonSlackLength_;
	return sqrt(first*first + second * second);
}

// Root solver for elasticTendonBiSec
template <typename T>
double wdbSolve(T &func, double x1, double x2, double tol) {

	const int ITMAX = 100;//original value = 100
	const double EPS = std::numeric_limits<double>::epsilon();

	double a = x1, b = x2, c = x2, d, e, fa = func(a), fb = func(b), fc, p, q, r, s, tol1, xm;
	if ((fa > 0.0 && fb > 0.0) || (fa < 0.0 && fb < 0.0))
		throw("Root must be bracketed in wdbSolve");
	fc = fb;
	for (int iter = 0; iter < ITMAX; iter++) {
		if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0)) {
			c = a;
			fc = fa;
			e = d = b - a;
		}
		if (fabs(fc) < fabs(fb)) {
			a = b;
			b = c;
			c = a;
			fa = fb;
			fb = fc;
			fc = fa;
		}

		tol1 = 2.0*EPS*fabs(b) + 0.5*tol;
		xm = 0.5*(c - b);
		if (fabs(xm) <= tol1 || fb == 0.0)
			return b;
		if (fabs(e) >= tol1 && fabs(fa) > fabs(fb)) {
			s = fb / fa;
			if (a == c) {
				p = 2.0*xm*s;
				q = 1.0 - s;
			}
			else {
				q = fa / fc;
				r = fb / fc;
				p = s * (2.0*xm*q*(q - r) - (b - a)*(r - 1.0));
				q = (q - 1.0)*(r - 1.0)*(s - 1.0);
			}
			if (p > 0.0)
				q = -q;
			p = fabs(p);
			double min1 = 3.0*xm*q - fabs(tol1*q);
			double min2 = abs(e*q);
			if (2.0*p < (min1 < min2 ? min1 : min2)) {
				e = d;
				d = p / q;
			}
			else {
				d = xm;
				e = d;
			}
		}
		else {
			d = xm;
			e = d;
		}
		a = b;
		fa = fb;
		if (fabs(d) > tol1)
			b += d;
		else
			b += sign(tol1, xm);
		fb = func(b);
	}
	throw("Maximum number of iterations exceeded in zbrent");
}

