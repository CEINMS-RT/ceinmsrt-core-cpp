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

#include "ComputeMuscleTendonLength.h"

ComputeMuscleTendonLength::ComputeMuscleTendonLength ( OpenSim::Model& unscaledModel, OpenSim::Model& scaledModel, MTSS& mtss, const std::string& translateFile ) :
	_scaledState ( scaledModel.initSystem() ), _unscaledState ( unscaledModel.initSystem() ), _unscaledModel ( unscaledModel ), _scaledModel ( scaledModel ), _mtss ( mtss )
{
	_unscaledMuscleAnalysis = new OpenSim::MuscleAnalysis ( &_unscaledModel );
	_unscaledMuscleAnalysis->setComputeMoments ( true );
	_scaledMuscleAnalysis = new OpenSim::MuscleAnalysis ( &_scaledModel );
	_scaledMuscleAnalysis->setComputeMoments ( true );

}

ComputeMuscleTendonLength::~ComputeMuscleTendonLength()
{
	delete _unscaledMuscleAnalysis;
	delete _scaledMuscleAnalysis;
}

void ComputeMuscleTendonLength::run()
{
	for ( MTSS::iterator it = _mtss.begin(); it < _mtss.end(); it++ )
	{
		{
			OpenSim::Array<std::string> DOFName;
			OpenSim::Array<std::string> muscleName;
			muscleName.append ( it->muscleName );

			for ( std::vector<std::string>::const_iterator itDOF = it->spanningDOF.begin(); itDOF < it->spanningDOF.end(); itDOF++ )
				DOFName.append ( *itDOF );

			OpenSim::Storage angleStorage;
			angleStorage.setName ( "AnglesData" );
			angleStorage.setInDegrees ( false );
			angleStorage.setColumnLabels ( DOFName );

			for ( int i = 0; i < 11; i++ )
			{
				//double tabState[DOFName.size()];
				//std::vector<double> tabState; //old
				SimTK::Vector_<double> tabState(it->postureControlPoints.size());
				//tabState.resize(it->postureControlPoints.size());

				for (std::vector<std::vector<double> >::const_iterator itPos = it->postureControlPoints.begin(); itPos < it->postureControlPoints.end(); itPos++)
					//tabState[std::distance<std::vector<std::vector<double> >::const_iterator> ( it->postureControlPoints.begin(), itPos )] = itPos->at ( i );
					//tabState.push_back(itPos->at(i)); old
					tabState[std::distance<std::vector<std::vector<double> >::const_iterator>(it->postureControlPoints.begin(), itPos)] = itPos->at(i);

				//OpenSim::StateVector stateVector ( i * 0.01, DOFName.size(), tabState.data() ); //old
				OpenSim::StateVector stateVector(i * 0.01, tabState);

				angleStorage.append ( stateVector, false );
			}

			_unscaledMuscleAnalysis->setMuscles ( muscleName );
			_unscaledMuscleAnalysis->setCoordinates ( DOFName );
			_unscaledMuscleAnalysis->setStatesStore ( angleStorage );

			for ( int i = 0; i < 11; i++ )
			{
				angleStorage.getTime ( i, _unscaledState.updTime() );
				setDOFUnscaled ( angleStorage, DOFName );
				_unscaledModel.getMultibodySystem().realize ( _unscaledState, SimTK::Stage::Dynamics );

				if ( i == 0 )
					_unscaledMuscleAnalysis->begin ( _unscaledState );
				else if ( i == 11 )
					_unscaledMuscleAnalysis->end ( _unscaledState );
				else
					_unscaledMuscleAnalysis->step ( _unscaledState, i );
			}

			OpenSim::Storage* lmtStorage = _unscaledMuscleAnalysis->getMuscleTendonLengthStorage();

			for ( int i = 0; i < DOFName.size(); i++ )
			{
				OpenSim::Array<double> colArray;
				lmtStorage->getDataColumn ( i, colArray );

				for ( int j = 0; j < colArray.size(); j++ )
					it->unscaledMuscleTendonLength.push_back ( colArray[j] );
			}
		}
		{
			OpenSim::Array<std::string> DOFName;
			OpenSim::Array<std::string> muscleName;
			muscleName.append ( it->muscleName );

			for ( std::vector<std::string>::const_iterator itDOF = it->spanningDOF.begin(); itDOF < it->spanningDOF.end(); itDOF++ )
				DOFName.append ( *itDOF  );

			OpenSim::Storage angleStorage;
			angleStorage.setName ( "AnglesData" );
			angleStorage.setInDegrees ( false );
			angleStorage.setColumnLabels ( DOFName );

			for ( int i = 0; i < 11; i++ )
			{
				//double tabState[DOFName.size()];
				//std::vector<double> tabState; //
				SimTK::Vector_<double> tabState(it->postureControlPoints.size());

				for ( std::vector<std::vector<double> >::const_iterator itPos = it->postureControlPoints.begin(); itPos < it->postureControlPoints.end(); itPos++ )
					//tabState[std::distance<std::vector<std::vector<double> >::const_iterator> ( it->postureControlPoints.begin(), itPos )] = itPos->at ( i );
					//tabState.push_back(itPos->at(i));
					tabState[std::distance<std::vector<std::vector<double> >::const_iterator>(it->postureControlPoints.begin(), itPos)] = itPos->at(i);

				//OpenSim::StateVector stateVector ( i * 0.01, DOFName.size(), tabState.data() );
				OpenSim::StateVector stateVector(i * 0.01, tabState);
				angleStorage.append ( stateVector, false );
			}

			_scaledMuscleAnalysis->setMuscles ( muscleName );
			_scaledMuscleAnalysis->setCoordinates ( DOFName );
			_scaledMuscleAnalysis->setStatesStore ( angleStorage );

			for ( int i = 0; i < 11; i++ )
			{
				angleStorage.getTime ( i, _scaledState.updTime() );
				setDOFScaled ( angleStorage, DOFName );
				_unscaledModel.getMultibodySystem().realize(_scaledState, SimTK::Stage::Dynamics);

				if ( i == 0 )
					_scaledMuscleAnalysis->begin ( _scaledState );
				else if ( i == 11 )
					_scaledMuscleAnalysis->end ( _scaledState );
				else
					_scaledMuscleAnalysis->step ( _scaledState, i );
			}

			OpenSim::Storage* lmtStorage = _scaledMuscleAnalysis->getMuscleTendonLengthStorage();

			for ( int i = 0; i < DOFName.size(); i++ )
			{
				OpenSim::Array<double> colArray;
				lmtStorage->getDataColumn ( i, colArray );

				for ( int j = 0; j < colArray.size(); j++ )
					it->scaledMuscleTendonLength.push_back ( colArray[j] );
			}
		}
	}
}

void ComputeMuscleTendonLength::setDOFUnscaled ( const OpenSim::Storage& angleStorage, const OpenSim::Array<std::string>& DOFName )
{
	//double tabValue[DOFName.size()];
	std::vector<double> tabValue(DOFName.size()); // make sure it reserve the data
	angleStorage.getDataAtTime ( _unscaledState.getTime(), DOFName.size(), tabValue.data() );

	for ( int i = 0 ; i < DOFName.size(); i++ )
		_unscaledModel.updCoordinateSet().get ( DOFName[i] ).setValue ( _unscaledState, tabValue[i] );

	_unscaledModel.assemble ( _unscaledState );
}

void ComputeMuscleTendonLength::setDOFScaled ( const OpenSim::Storage& angleStorage, const OpenSim::Array<std::string>& DOFName )
{
	//double tabValue[DOFName.size()];
	std::vector<double> tabValue(DOFName.size());
	angleStorage.getDataAtTime ( _scaledState.getTime(), DOFName.size(), tabValue.data() );

	for ( int i = 0 ; i < DOFName.size(); i++ )
		_scaledModel.updCoordinateSet().get ( DOFName[i] ).setValue ( _scaledState, tabValue[i] );

	_scaledModel.assemble ( _scaledState );
}
