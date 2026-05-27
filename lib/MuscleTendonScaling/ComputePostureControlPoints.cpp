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
