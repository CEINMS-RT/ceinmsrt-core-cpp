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
