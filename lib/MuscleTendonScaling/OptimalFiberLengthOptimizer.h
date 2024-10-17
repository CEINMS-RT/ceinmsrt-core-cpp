/*
 * Copyright (c) 2015, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
