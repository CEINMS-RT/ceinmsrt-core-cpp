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

#ifndef ElasticTendon_BiSec_h
#define ElasticTendon_BiSec_h
#include <string>
#include "Curve.h"

//template <typename CurveMode::Mode mode>
template <typename CurveM>
class ElasticTendon_BiSec {

public:
	ElasticTendon_BiSec();
	ElasticTendon_BiSec(std::string id);
	ElasticTendon_BiSec(double optimalFibreLength,
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
	virtual ~ElasticTendon_BiSec() {}
	double operator()(double fl);
	ElasticTendon_BiSec(const ElasticTendon_BiSec<CurveM>& orig);
	ElasticTendon_BiSec<CurveM>& operator=(const ElasticTendon_BiSec<CurveM>& orig);

	void setParametersToComputeForces(double optimalFibreLength,
		double pennationAngle,
		double tendonSlackLength,
		double percentageChange,
		double damping,
		double maxIsometricForce,
		double strengthCoefficient);

	void setTime(const double& time);
	void setMuscleTendonLength(double muscleTendonLength);
	void setActivation(double activation);
	void setOptimalFibreLength(double optimalFibreLength) {
		optimalFibreLength_ = optimalFibreLength;
		resetState();
	}
	void updateFibreLength();

	double getFibreLength() { return fibreLength_; }
	void setStrengthCoefficient(double strengthCoefficient);
	void setTendonSlackLength(double tendonSlackLength);
	void setCurves(const CurveM& activeForceLengthCurve,
		const CurveM& passiveForceLengthCurve,
		const CurveM& forceVelocityCurve);

	void pushState();
	void resetState();
	double getTendonLength() const { return tendonLength_; };
	double getMuscleTendonLength() const { return muscleTendonLength_; };

	double computeNormalizedTendonStiffness(double tendonStrain);
	/*

	void setMuscleTendonLength(double muscleTendonLength, double activation, double time);
	void updateFiberLengthUsingNewActivation(double activation, double time);
	double getFiberLength() { return fibreLength_;}
	void setStrengthCoefficient(double strengthCoefficient);
	void setTendonSlackLength(double tendonSlackLength);
	void setCurves(const CurveOffline& activeForceLengthCurve,
				   const CurveOffline& passiveForceLengthCurve,
				   const CurveOffline& forceVelocityCurve);

	void resetState();
	*/

private:
	double estimateFiberLengthBiSec(double tol, unsigned maxIterations);

	double evaluateForceError(double fibreLength);
	double computeMuscleForce(double fibreLength);
	double computeTendonForce(double fibreLength);
	double getFibreLengthStiff() const;

	double optimalFibreLength_;
	double pennationAngle_;
	double tendonSlackLength_;
	double percentageChange_;
	double damping_;
	double maxIsometricForce_;
	double strengthCoefficient_;
	CurveM activeForceLengthCurve_;
	CurveM passiveForceLengthCurve_;
	CurveM forceVelocityCurve_;
	CurveM tendonForceStrainCurve_;


	CurveM  fibreLengthTrace_;
	double time_;
	double timeScale_;

	double fibreLength_;
	double fibreLengthT1_;          //valore della fiberLength al passo precedente
	double muscleTendonLength_;
	double activation_;
	double tendonLength_;
	double tendonPenalty_;
	double tolerance_;
	std::string id_;



};


#include "ElasticTendon_BiSec.cpp"

#endif
