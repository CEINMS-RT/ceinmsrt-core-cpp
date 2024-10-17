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

#include "ComputePostureControlPoints.h"

#ifdef USE_OPENSIM

ComputePostureControlPoints::ComputePostureControlPoints ( const std::string& xmlFile, const OpenSim::Model& unscaledModel, MTSS& mtss, const std::string& translateFile  ) :
	_unscaledModel ( unscaledModel )
{
	DOFMuscleMap DOFMuscleConnect;
	std::set<std::string> muscleNameSet;

	try
	{
		std::auto_ptr<NMSmodelType> subjectPointer ( subject ( xmlFile ) );

		// DOF iteration
		NMSmodelType::DoFs_type& dofs ( subjectPointer->DoFs() );
		DoFsType::DoF_sequence& dofSequence ( dofs.DoF() );

		for ( DoFsType::DoF_iterator i = dofSequence.begin(); i != dofSequence.end(); ++i )
		{
			std::string currentDOF ( ( *i ).name() );

			// iteration in the muscle containing in each DOF. See end of subject specific XML.
			MuscleSequenceType currentSequence = ( *i ).muscleSequence();

			for ( MuscleSequenceType::iterator muscleIt = currentSequence.begin(); muscleIt != currentSequence.end(); ++muscleIt )
			{
				DOFMuscleConnect.insert ( position ( currentDOF, *muscleIt ) );
				muscleNameSet.insert ( *muscleIt );
			}
		}

		mtss.resize ( muscleNameSet.size() );

		for ( std::set<std::string>::const_iterator itSet = muscleNameSet.begin(); itSet != muscleNameSet.end(); itSet++ )
			mtss.at ( std::distance< std::set<std::string>::const_iterator > ( muscleNameSet.begin(), itSet ) ).muscleName = *itSet;

		NMSmodelType::muscles_type& muscles ( subjectPointer->muscles() );
		MusclesType::muscle_sequence& muscleSequence ( muscles.muscle() );

		for ( MusclesType::muscle_iterator i ( muscleSequence.begin() ); i != muscleSequence.end(); ++i )
		{
			std::set<std::string>::iterator itNameSet = muscleNameSet.find ( i->name() );

			if ( itNameSet != muscleNameSet.end() )
			{
				const int& cpt = std::distance<std::set<std::string>::iterator> ( muscleNameSet.begin(), itNameSet );
				mtss.at ( cpt ).optimalPennationAngle = i->pennationAngle();
				mtss.at ( cpt ).unscaledTendonSlackLength = i->tendonSlackLength();
				mtss.at ( cpt ).unscaledOptimalFiberLength = i->optimalFiberLength();
			}
		}

	}
	catch ( const xml_schema::exception& e )
	{
		std::cout << e << std::endl;
		exit ( EXIT_FAILURE );
	}

	for ( MTSS::iterator it = mtss.begin(); it < mtss.end(); it++ )
		for ( DOFMuscleMap::right_const_iterator itBimap = DOFMuscleConnect.right.lower_bound ( it->muscleName ); itBimap != DOFMuscleConnect.right.upper_bound ( it->muscleName ); itBimap++ )
		{
			it->spanningDOF.push_back ( itBimap->second );
			double lower = getMinAnglesDof (  itBimap->second );
			double higther = getMaxAnglesDof (  itBimap->second );
			double step = (higther - lower) / 10;
			std::vector<double> postureControlPoints;
			postureControlPoints.push_back ( lower );

			for ( int i = 1; i < 10; i++ )
				postureControlPoints.push_back ( lower + i * step );

			postureControlPoints.push_back ( higther );
			it->postureControlPoints.push_back ( postureControlPoints );
		}
}

ComputePostureControlPoints::~ComputePostureControlPoints()
{
}

double ComputePostureControlPoints::getMinAnglesDof ( const std::string& nameOfDOF ) const
{
	const OpenSim::CoordinateSet& coordinateSet = _unscaledModel.getCoordinateSet();
	return coordinateSet.get ( nameOfDOF ).getRangeMin();
}

double ComputePostureControlPoints::getMaxAnglesDof ( const std::string& nameOfDOF ) const
{
	const OpenSim::CoordinateSet& coordinateSet = _unscaledModel.getCoordinateSet();
	return coordinateSet.get ( nameOfDOF ).getRangeMax();
}

#endif
