// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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

