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

#ifndef COMPUTEPOSTURECONTROLPOINTS_H
#define COMPUTEPOSTURECONTROLPOINTS_H

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include <boost/shared_ptr.hpp>
#include "NMSmodel.hxx"
#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>

struct MuscleTendonScalingStruct
{
	std::string muscleName;
	std::vector<std::string> spanningDOF;
	std::vector<std::vector<double> > postureControlPoints;
	std::vector<double> unscaledMuscleTendonLength;
	std::vector<double> scaledMuscleTendonLength;
	std::vector<double> fiberLength;
	double optimalPennationAngle;
	double unscaledTendonSlackLength;
	double scaledTendonSlackLength;
	double unscaledOptimalFiberLength;
	double scaledOptimalFiberLength;
};

typedef std::vector<MuscleTendonScalingStruct> MTSS;

class ComputePostureControlPoints
{
	public:
		ComputePostureControlPoints ( const std::string& xmlFile, const OpenSim::Model& unscaledModel, MTSS& mtss, const std::string& translateFile );
		~ComputePostureControlPoints();
		
		/**
		 * Return the max angles range for the coordinate.
		 * @param nameOfDOF OpenSim name of the DOF
		 */
		double getMinAnglesDof ( const std::string& nameOfDOF ) const;

		/**
		 * Return the min angles range for the coordinate.
		 * @param nameOfDOF OpenSim name of the DOF
		 */
		double getMaxAnglesDof ( const std::string& nameOfDOF ) const;
		
	protected:
		typedef boost::bimap < boost::bimaps::multiset_of<std::string>, boost::bimaps::multiset_of<std::string> > DOFMuscleMap;
		typedef DOFMuscleMap::value_type position;
		const OpenSim::Model& _unscaledModel;
};

#endif

#endif // COMPUTEPOSTURECONTROLPOINTS_H
