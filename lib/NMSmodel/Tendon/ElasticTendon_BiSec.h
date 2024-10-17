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
