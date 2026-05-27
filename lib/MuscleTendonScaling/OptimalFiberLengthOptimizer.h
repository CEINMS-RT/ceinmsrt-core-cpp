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

#ifndef OPTIMALFIBERLENGTHOPTIMIZER_H
#define OPTIMALFIBERLENGTHOPTIMIZER_H

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include "Curve.h"
#include "NMSmodel.hxx"
#include "ComputeMuscleTendonLength.h"
#include <cmath>

class OptimalFiberLengthOptimizer : public SimTK::OptimizerSystem
{
	public:
		OptimalFiberLengthOptimizer ( const std::string& xmlModelFile, MuscleTendonScalingStruct& mtss, int pointsControlNumber );
		~OptimalFiberLengthOptimizer();

		int objectiveFunc ( const SimTK::Vector& newControls, const bool new_coefficients, SimTK::Real& f ) const
		{
// 			std::cout << _mtss.muscleTendonLength.size() << " : " << _pointsControlNumber << std::endl;
// 			std::cout << "1: " << newControls[0] << std::endl;

			double alpha = ( sin ( _mtss.optimalPennationAngle ) ) / newControls[0];

// 			std::cout << "alpha: " << alpha << " _mtss.optimalPennationAngle: " << _mtss.optimalPennationAngle << std::endl;

			double cosAlpha;

			if ( alpha <= 0 )
				cosAlpha = 1;
			else if ( alpha >= 1 )
				cosAlpha = 0;
			else
				cosAlpha = cos ( asin ( alpha ) ) ;

// 			std::cout << "cosAlpha: " << cosAlpha << std::endl;

			double NormActiveMuscleForce = _activeForceLengthCurve.getValue ( newControls[0] );

			if ( NormActiveMuscleForce < 0 )
				NormActiveMuscleForce = 0;

			double NormPassiveMuscleForce = _passiveForceLengthCurve.getValue ( newControls[0] );

			if ( NormPassiveMuscleForce < 0 )
				NormPassiveMuscleForce = 0;

			double NormMuscleForce = NormActiveMuscleForce + NormPassiveMuscleForce;

// 			std::cout << "3: " << NormActiveMuscleForce << " + " << NormPassiveMuscleForce << " = " << NormMuscleForce << std::endl;

			if ( NormMuscleForce * cosAlpha < 0.23875 )
			{
// 				std::cout << "4" << std::endl;
				f = _mtss.unscaledTendonSlackLength + _mtss.unscaledTendonSlackLength * log ( ( NormMuscleForce * cosAlpha / 0.06142 )
						+ 1 ) / 124.929 - _mtss.unscaledMuscleTendonLength[_pointsControlNumber] + _mtss.unscaledOptimalFiberLength
						* newControls[0] * cosAlpha;

// 				std::cout << _mtss.unscaledTendonSlackLength   - _mtss.muscleTendonLength[_pointsControlNumber]  << " + " <<  _mtss.unscaledTendonSlackLength* log ( ( NormMuscleForce * cosAlpha / 0.06142 )
// 						+ 1 ) / 124.929  +  _mtss.unscaledOptimalFiberLength
// 						* newControls[0] * cosAlpha << std::endl;
			}
			else
			{
// 				std::cout << "5" << std::endl;
				f = _mtss.unscaledTendonSlackLength + 0.2375 * _mtss.unscaledTendonSlackLength / 37.5
						- _mtss.unscaledMuscleTendonLength[_pointsControlNumber] + _mtss.unscaledTendonSlackLength
						* NormMuscleForce * cosAlpha / 37.5 + _mtss.unscaledOptimalFiberLength * newControls[0] * cosAlpha;

// 				std::cout << _mtss.unscaledTendonSlackLength + 0.2375 * _mtss.unscaledTendonSlackLength / 37.5
// 						-  _mtss.muscleTendonLength[_pointsControlNumber] << " + " << _mtss.unscaledTendonSlackLength
// 						* NormMuscleForce* cosAlpha / 37.5 + _mtss.unscaledOptimalFiberLength* newControls[0] * cosAlpha << std::endl;
			}

// 			f = abs ( f );
			f = f*f;
// 			std::cout << "f = " << f << std::endl;
// 			std::cout << std::endl;

			return 0;

		}


	protected:
		typedef Curve<CurveMode::Offline> CurveOffline;
		CurveOffline _forceVelocityCurve;
		CurveOffline _activeForceLengthCurve;
		CurveOffline _passiveForceLengthCurve;
		std::auto_ptr<NMSmodelType> _subjectPointer;
		int _pointsControlNumber;

		MuscleTendonScalingStruct& _mtss;

		void createCurves();
};

#endif

#endif // OPTIMALFIBERLENGTHOPTIMIZER_H
