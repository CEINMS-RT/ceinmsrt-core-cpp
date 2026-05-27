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

#ifndef TSLANDOFLOPTIMIZER_H
#define TSLANDOFLOPTIMIZER_H

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include "Curve.h"
#include "NMSmodel.hxx"
#include "ComputeMuscleTendonLength.h"
#include <cmath>

class TSLAndOFLOptimizer : public SimTK::OptimizerSystem
{
	public:
		TSLAndOFLOptimizer ( const std::string& xmlModelFile, MuscleTendonScalingStruct& mtss );
		~TSLAndOFLOptimizer();

		int objectiveFunc ( const SimTK::Vector& newControls, const bool new_coefficients, SimTK::Real& f ) const
		{
			f = 0;
			double result;
			
//  			std::cout << "lts: " << newControls[0] << " -- " << "lof" << newControls[1] << endl;

			for ( int i = 0; i < 11; i++ )
			{
				double alpha = ( sin ( _mtss.optimalPennationAngle ) ) / _mtss.fiberLength.at ( i );

				double cosAlpha;

				if ( alpha <= 0 )
					cosAlpha = 1;
				else if ( alpha >= 1 )
					cosAlpha = 0;
				else
					cosAlpha = cos ( asin ( alpha ) ) ;

				double NormActiveMuscleForce = _activeForceLengthCurve.getValue ( _mtss.fiberLength.at ( i ) );

				if ( NormActiveMuscleForce < 0 )
					NormActiveMuscleForce = 0;

				double NormPassiveMuscleForce = _passiveForceLengthCurve.getValue ( _mtss.fiberLength.at ( i ) );

				if ( NormPassiveMuscleForce < 0 )
					NormPassiveMuscleForce = 0;

				double NormMuscleForce = NormActiveMuscleForce + NormPassiveMuscleForce;

				if ( NormMuscleForce * cosAlpha < 0.23875 )
				{
					result = _mtss.scaledMuscleTendonLength[i] - ( newControls[0] + newControls[0]
							* log ( ( NormMuscleForce * cosAlpha / 0.06142 )
									+ 1 ) / 124.929 + newControls[1]
							* _mtss.fiberLength.at ( i ) * cosAlpha );
					
// 					std::cout << ( newControls[0] + newControls[0]
// 							* log ( ( NormMuscleForce * cosAlpha / 0.06142 )
// 									+ 1 ) / 124.929 + newControls[1]
// 							* _mtss.fiberLength.at ( i ) * cosAlpha ) << std::endl;
				}
				else
				{
					result = _mtss.scaledMuscleTendonLength[i] - ( newControls[0] + 0.2375
							* newControls[0] / 37.5 + newControls[0]
							* NormMuscleForce * cosAlpha / 37.5
							+ newControls[1] * _mtss.fiberLength.at ( i ) * cosAlpha );
					
// 					std::cout << ( newControls[0] + 0.2375
// 							* newControls[0] / 37.5 + newControls[0]
// 							* NormMuscleForce * cosAlpha / 37.5
// 							+ newControls[1] * _mtss.fiberLength.at ( i ) * cosAlpha ) <<std::endl;
				}
				result = result * result;
				
// 				std::cout << std::endl;
// 				result = abs ( result );
			}

			f = f + result;
			
//  			std::cout << "f= " << f << std::endl;
// 			std::cout << std::endl;
			
			return 0;
		}

	protected:
		typedef Curve<CurveMode::Offline> CurveOffline;
		CurveOffline _forceVelocityCurve;
		CurveOffline _activeForceLengthCurve;
		CurveOffline _passiveForceLengthCurve;
		std::auto_ptr<NMSmodelType> _subjectPointer;

		MuscleTendonScalingStruct& _mtss;

		void createCurves();
};

#endif

#endif // TSLANDOFLOPTIMIZER_H
