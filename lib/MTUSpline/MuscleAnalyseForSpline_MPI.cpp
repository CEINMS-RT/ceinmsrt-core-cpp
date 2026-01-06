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

#include "MuscleAnalyseForSpline_MPI.h"
//#define PRINT_DOF_LIST

#define PRECISION 6

#ifdef USE_OPENSIM
#ifdef MPI

MuscleAnalyseForSplineMPI::MuscleAnalyseForSplineMPI ( string& configurationFile, string& OsimModelName,
		int nbOfStep ) : MuscleAnalyseForSpline ( configurationFile, OsimModelName, nbOfStep, 0 )
{

}
MuscleAnalyseForSplineMPI::MuscleAnalyseForSplineMPI ( string& configurationFile, string& OsimModelName,
		const string& translateFileName, int nbOfStep ) :
	MuscleAnalyseForSpline ( configurationFile, OsimModelName, translateFileName, nbOfStep, 0 )
{

}

MuscleAnalyseForSplineMPI::~MuscleAnalyseForSplineMPI()
{
}

void MuscleAnalyseForSplineMPI::run ( const int& rank, const int& ncpu )
{
	for ( vector<Task>::iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++ )
	{
		std::cout << "Task " << std::distance<vector<Task>::iterator> ( taskVect_.begin(), it1 ) << std::endl;

		if ( rank == 0 )
		{
			int j = 0, overflowCPU = -1;

			while ( true )
			{
				std::vector<double> lmtData;
				lmtData.resize ( it1->uniqueMuscleList.size() );
				it1->lmtVectorMat.resize ( it1->uniqueMuscleList.size() );

				for ( int i = 1; i < ncpu; i++ )
				{
					MPI_Send ( &j, 1, MPI_INT, i, DATA_REQ, MPI_COMM_WORLD );
					j++;

					if ( j > it1->angleStorage.getSize() )
					{
						overflowCPU = i;
						break;
					}
				}

				for ( int i = 1; i < ncpu; i++ )
				{
					MPI_Recv ( lmtData.data(), it1->uniqueMuscleList.size(), MPI_DOUBLE, i, DATA_RES, MPI_COMM_WORLD, MPI_STATUS_IGNORE );

					for ( std::vector<double>::const_iterator itData = lmtData.begin(); itData < lmtData.end(); itData++ )
						it1->lmtVectorMat[std::distance<std::vector<double>::const_iterator> ( lmtData.begin(), itData )].push_back ( *itData );

					if ( i == overflowCPU )
						break;
				}

				if ( j > it1->angleStorage.getSize() )
				{
					for ( int i = 1; i < ncpu; i++ )
						MPI_Send ( &j, 1, MPI_INT, i, DATA_REQ, MPI_COMM_WORLD );

					std::cout << "Next Task" << std::endl;
					break;
				}
			}
		}
		else
		{
			si_ = osimModel_.initSystem();
			osimModel_.equilibrateMuscles ( si_ );
			
			OpenSim::ForceSet& fSet = osimModel_.updForceSet();

			std::vector<OpenSim::Muscle*> muscleVect;

			for ( set<string>::const_iterator it2 = it1->uniqueMuscleList.begin(); it2 != it1->uniqueMuscleList.end(); it2++ )
			{
				if ( fSet.contains ( *it2 ) )
				{
					OpenSim::Muscle* mus = dynamic_cast<OpenSim::Muscle*> ( &fSet.get ( *it2 ) );

					if ( mus )
					{
						muscleVect.push_back ( mus );
					}
				}
			}

			while ( true )
			{
				int i;
				MPI_Recv ( &i, 1, MPI_INT, 0, DATA_REQ, MPI_COMM_WORLD, MPI_STATUS_IGNORE );

				if ( i > it1->angleStorage.getSize() )
				{
					std::cout << "Next Task" << std::endl;
					break;
				}

				( *it1 ).angleStorage.getTime ( i, si_.updTime() ); // time
				setDOF ( *it1 );
				osimModel_.getMultibodySystem().realize ( si_, SimTK::Stage::Dynamics );
// 				const OpenSim::Set< OpenSim::Muscle >& muscles = osimModel_.getMuscles();
// 				osimModel_.equilibrateMuscles ( si_ );
				std::vector<double> lmtData;
// 				it1->lmtVectorMat.resize ( it1->uniqueMuscleList.size() );
// 
				for ( std::vector<OpenSim::Muscle*>::const_iterator it2 = muscleVect.begin(); it2 != muscleVect.end(); it2++ )
					lmtData.push_back ( ( *it2 )->getLength ( si_ ) );
				
// 				std::cout << lmtData.at ( 0 ) << std::endl;

				MPI_Send ( lmtData.data(), lmtData.size(), MPI_DOUBLE, 0, DATA_RES, MPI_COMM_WORLD );
			}
		}
	}
}

void MuscleAnalyseForSplineMPI::setDOF ( const Task& task )
{
	double tabValue[task.uniqueDOFlist.size()];
	task.angleStorage.getDataAtTime ( si_.getTime(), task.uniqueDOFlist.size(), tabValue );
// 	std::cout << tabValue[0] << std::endl;

	for ( set<string>::const_iterator it = task.uniqueDOFlist.begin(); it != task.uniqueDOFlist.end(); it++ )
		osimModel_.updCoordinateSet().get ( *it ).setValue ( si_, tabValue[distance<set<string>::const_iterator> ( task.uniqueDOFlist.begin(), it )] );
	
 	osimModel_.updateAssemblyConditions( si_ );
 	osimModel_.assemble ( si_ );
}


#endif
#endif

