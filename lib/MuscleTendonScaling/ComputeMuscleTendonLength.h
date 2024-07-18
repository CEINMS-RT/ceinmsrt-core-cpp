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

#ifndef COMPUTEMUSCLETENDONLENGTH_H
#define COMPUTEMUSCLETENDONLENGTH_H

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include "ComputePostureControlPoints.h"

class ComputeMuscleTendonLength
{
	public:
		ComputeMuscleTendonLength ( OpenSim::Model& unscaledModel, OpenSim::Model& scaledModel, MTSS& mtss, const std::string& translateFile );
		~ComputeMuscleTendonLength();
		void run();
	protected:
		SimTK::State& _unscaledState;
		SimTK::State& _scaledState;
		OpenSim::Model& _unscaledModel;
		OpenSim::Model& _scaledModel;
		OpenSim::MuscleAnalysis* _unscaledMuscleAnalysis;
		OpenSim::MuscleAnalysis* _scaledMuscleAnalysis;
		MTSS& _mtss;
		
		void setDOFUnscaled(const OpenSim::Storage& angleStorage, const OpenSim::Array<std::string>& DOFName);
		void setDOFScaled(const OpenSim::Storage& angleStorage, const OpenSim::Array<std::string>& DOFName);
};

#endif

#endif // COMPUTEMUSCLETENDONLENGTH_H
