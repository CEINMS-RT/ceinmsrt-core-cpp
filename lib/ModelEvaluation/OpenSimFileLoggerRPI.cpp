/*
 * Copyright (c) 2016, <copyright holder> <email>
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

#include "OpenSimFileLogger.h"

template <typename NMSmodelT>
OpenSimFileLogger<NMSmodelT>::OpenSimFileLogger ( const NMSmodelT& subjectModel, const std::string& recordDirectory ) :
	_subjectModel ( subjectModel ), _subjectModelGiven ( true ), _recordDirectory ( recordDirectory ), _cptMarker ( 0 ), _cptMarkerFilter ( 0 )
{
	// Create the directory for the recorded file
	std::stringstream ss;
	ss << "./";
	ss << _recordDirectory;
	boost::filesystem::path dir ( ss.str().c_str() );

	if ( !boost::filesystem::exists ( dir ) )
		if ( !boost::filesystem::create_directory ( dir ) )
			COUT << "ERROR in creating directory: " << ss.str() << std::endl;
}

template <typename NMSmodelT>
OpenSimFileLogger<NMSmodelT>::OpenSimFileLogger ( const std::string& recordDirectory ) :
	_subjectModelGiven ( false ), _recordDirectory ( recordDirectory ), _cptMarker ( 0 ), _cptMarkerFilter ( 0 ), _subjectModel ( NMSmodelT() )
{
	// Create the directory for the recorded file
	std::stringstream ss;
	ss << "./";
	ss << _recordDirectory;
	boost::filesystem::path dir ( ss.str().c_str() );

	if ( !boost::filesystem::exists ( dir ) )
		if ( !boost::filesystem::create_directory ( dir ) )
			COUT << "ERROR in creating directory: " << ss.str() << std::endl;
}

template <typename NMSmodelT>
OpenSimFileLogger<NMSmodelT>::~OpenSimFileLogger()
{
	for ( std::map<Logger::LogID, std::ofstream*>::iterator it  = _mapLogIDToFile.begin(); it != _mapLogIDToFile.end(); it++ )
		delete it->second;

	for ( std::map<std::string, std::ofstream*>::iterator it  = _mapMANametoFile.begin(); it != _mapMANametoFile.end(); it++ )
		delete it->second;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::addLog ( Logger::LogID logID, const std::vector<std::string>& ColumnName )
{
	using namespace Logger;
	std::string filename;
	std::stringstream ss;

	switch ( logID )
	{
		case Activations:
			filename = "/Activations.sto";
			_mapLogIDToNumerOfRow[Activations] = 0;
			break;
			
		case MotorTorque:
			filename = "/MotorTorque.sto";
			_mapLogIDToNumerOfRow[MotorTorque] = 0;
			break;

		case FibreLengths:
			filename = "/FibreLengths.sto";
			_mapLogIDToNumerOfRow[FibreLengths] = 0;
			break;
			
		case FootSwitch:
			filename = "/FootSwitch.sto";
			_mapLogIDToNumerOfRow[FootSwitch] = 0;
			break;

		case FibreVelocities:
			filename = "/FibreVelocities.sto";
			_mapLogIDToNumerOfRow[FibreVelocities] = 0;
			break;

		case MuscleForces:
			filename = "/MuscleForces.sto";
			_mapLogIDToNumerOfRow[MuscleForces] = 0;
			break;

		case LMT:
			filename = "/lmt.sto";
			_mapLogIDToNumerOfRow[LMT] = 0;
			break;

		case Torques:
			filename = "/Torque.sto";
			_mapLogIDToNumerOfRow[Torques] = 0;
			break;
			
		case TorquesFilt:
			filename = "/TorquesFilt.sto";
			_mapLogIDToNumerOfRow[TorquesFilt] = 0;
			break;

		case Logger::PennationAngle:
			filename = "/PennationAngle.sto";
			_mapLogIDToNumerOfRow[Logger::PennationAngle] = 0;
			break;

		case TendonLength:
			filename = "/TendonLength.sto";
			_mapLogIDToNumerOfRow[TendonLength] = 0;
			break;

		case Emgs:
			filename = "/emg.sto";
			_mapLogIDToNumerOfRow[Emgs] = 0;
			break;

		case EmgsFilter:
			filename = "/emgFilt.sto";
			_mapLogIDToNumerOfRow[EmgsFilter] = 0;
			break;

		case Marker:
			filename = "/marker.trc";
			_mapLogIDToNumerOfRow[Marker] = 0;
			break;

		case MarkerFilter:
			filename = "/markerFilt.trc";
			_mapLogIDToNumerOfRow[MarkerFilter] = 0;
			break;

		case ForcePlate:
			filename = "/forcePlate.sto";
			_mapLogIDToNumerOfRow[ForcePlate] = 0;
			break;

		case ForcePlateFilter:
			filename = "/forcePlateFilt.sto";
			_mapLogIDToNumerOfRow[ForcePlateFilter] = 0;
			break;

		case IK:
			filename = "/ik.sto";
			_mapLogIDToNumerOfRow[IK] = 0;
			break;

		case ID:
			filename = "/id.sto";
			_mapLogIDToNumerOfRow[ID] = 0;
			break;

		case NMSTiming:
			filename = "/NMSTimming.csv";
			break;

		case TotalTiming:
			filename = "/TotalTimming.csv";
			break;

		case IKTiming:
			filename = "/IKTiming.csv";
			break;

		case MTUTiming:
			filename = "/MTUTiming.csv";
			break;
			
		case Battery:
			filename = "/Battery.csv";
			break;
	}

	ss << "./";
	ss << _recordDirectory;
	ss << filename;
	_mapLogIDToFile[logID] = new std::ofstream ( ss.str().c_str() );
	_mapLogIDToVect[logID] =  std::vector<std::vector<double> >();
	std::ofstream* file = _mapLogIDToFile[logID];

	if ( ! ( file->is_open() ) )
		COUT << "ERROR: " + ss.str() + " cannot be opened!" << std::endl;

	HeaderFile headerFile;
	headerFile.setNumberOfRow ( 0 );
	headerFile.setNumberOfColumn ( ColumnName.size() + 1 );// +1 for time
	headerFile.setNameOfColumn ( ColumnName );
	headerFile.setInDegrees ( false );

	switch ( logID )
	{
		case Activations:
			headerFile.writeFile ( *file, ss.str(), "Activations" );
			break;
			
		case FootSwitch:
			headerFile.writeFile ( *file, ss.str(), "FootSwitch" );
			break;
			
		case MotorTorque:
			headerFile.writeFile ( *file, ss.str(), "MotorTorque" );
			break;

		case FibreLengths:
			headerFile.writeFile ( *file, ss.str(), "FibreLength" );
			break;

		case FibreVelocities:
			headerFile.writeFile ( *file, ss.str(), "FibreVelocities" );
			break;

		case MuscleForces:
			headerFile.writeFile ( *file, ss.str(), "MuscleForces" );
			break;

		case Torques:
			headerFile.writeFile ( *file, ss.str(), "Torques" );
			break;
			
		case TorquesFilt:
			headerFile.writeFile ( *file, ss.str(), "TorquesFilt" );
			break;


		case LMT:
			headerFile.writeFile ( *file, ss.str(), "LMT" );
			break;

		case Logger::PennationAngle:
			headerFile.writeFile ( *file, ss.str(), "PennationAngle" );
			break;

		case TendonLength:
			headerFile.writeFile ( *file, ss.str(), "TendonLength" );
			break;

		case Emgs:
			headerFile.writeFile ( *file, ss.str(), "Emgs" );
			break;

		case EmgsFilter:
			headerFile.writeFile ( *file, ss.str(), "EmgsFilter" );
			break;

		case ForcePlate:
			headerFile.writeFile ( *file, ss.str(), "ForcePlate" );
			break;

		case ForcePlateFilter:
			headerFile.writeFile ( *file, ss.str(), "ForcePlateFilter" );
			break;

		case IK:
			headerFile.writeFile ( *file, ss.str(), "IK" );
			break;

		case ID:
			headerFile.writeFile ( *file, ss.str(), "ID" );
			break;

		case Marker:
			_mapLogIDToNumerOfRow[Marker] = 0;
			*file << "PathFileType	4	(X/Y/Z)	marker.trc" << std::endl;
			markerHearder ( *file, ColumnName, 0 );
			columnMarkerNames_ = ColumnName;
			break;

		case MarkerFilter:
			_mapLogIDToNumerOfRow[MarkerFilter] = 0;
			*file << "PathFileType	4	(X/Y/Z)	MarkerFilter.trc" << std::endl;
			markerHearder ( *file, ColumnName, 0 );
			columnMarkerNames_ = ColumnName;
			break;
	}

}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::addLog ( Logger::LogID logID )
{
	using namespace Logger;
	std::string filename;
	std::stringstream ss;

	switch ( logID )
	{
		case NMSTiming:
			filename = "/NMSTimming.csv";
			break;

		case TotalTiming:
			filename = "/TotalTimming.csv";
			break;

		case IKTiming:
			filename = "/IKTiming.csv";
			break;

		case MTUTiming:
			filename = "/MTUTiming.csv";
			break;
			
		case Battery:
			filename = "/Battery.csv";
			break;

		default:
			COUT << "Error wong addlog method for " << logID << std::endl;
			break;
	}

	ss << "./";
	ss << _recordDirectory;
	ss << filename;
	_mapLogIDToFile[logID] = new std::ofstream ( ss.str().c_str() );
	_mapLogIDToVect[logID] =  std::vector<std::vector<double> >();

	if ( ! ( _mapLogIDToFile[logID]->is_open() ) )
		COUT << "ERROR: " + ss.str() + " cannot be opened!" << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::markerHearder ( std::ofstream& file, const std::vector<std::string>& ColumnName, const unsigned int& numbersOfFrames )
{
	file << "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames" << std::endl;
	file << "128\t128\t" << numbersOfFrames << "\t" << ColumnName.size() << "\tm\t128\t1\t" << numbersOfFrames << std::endl;
	file << "Frame#\ttime\t";

	for ( std::vector<std::string>::const_iterator it = ColumnName.begin(); it != ColumnName.end(); it++ )
		file << *it << "\t";

	file << std::endl;
	file << "\t\t";

	for ( int cpt = 0; cpt < ColumnName.size(); cpt++ )
	{
		file << "x" << cpt << "\t";
		file << "y" << cpt << "\t";
		file << "z" << cpt << "\t";
	}

	file << std::endl;
	file << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::addMa ( const std::string& maName, const std::vector<std::string>& ColumnName )
{
	using namespace Logger;
	std::stringstream ss;
	HeaderFile headerFile;
	ss << "./";
	ss << _recordDirectory;
	ss << "/ma_" << maName << ".sto";
	_mapMANametoFile[maName] = new std::ofstream ( ss.str().c_str() );
	std::ofstream* file = _mapMANametoFile[maName];
	_mapMANametoNumerOfRow[maName] = 0;
	
	_mapMANametoVect[maName] =  std::vector<std::vector<double> >();
	headerFile.setNumberOfRow ( 0 );
	headerFile.setNumberOfColumn ( ColumnName.size() + 1 ); // +1 for time
	headerFile.setNameOfColumn ( ColumnName );
	headerFile.setInDegrees ( false );
	headerFile.writeFile ( *file, "ma_" + maName + ".sto", "ma_" + maName );
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time )
{
	using namespace Logger;
// 	std::ofstream* file = _mapLogIDToFile.at ( logID );
	std::vector<double> data;

	if ( _subjectModelGiven )
		switch ( logID )
		{
			case Activations:
				_subjectModel.getActivations ( data );
				_mapLogIDToNumerOfRow[Activations]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case FibreLengths:
				_subjectModel.getFiberLengths ( data );
				_mapLogIDToNumerOfRow[FibreLengths]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case FibreVelocities:
				_subjectModel.getFiberVelocities ( data );
				_mapLogIDToNumerOfRow[FibreVelocities]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case MuscleForces:
				_subjectModel.getMuscleForces ( data );
				_mapLogIDToNumerOfRow[MuscleForces]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case Torques:
				_subjectModel.getTorques ( data );
				_mapLogIDToNumerOfRow[Torques]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case Logger::PennationAngle:
				_subjectModel.getPennationAngleInst ( data );
				_mapLogIDToNumerOfRow[Logger::PennationAngle]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case TendonLength:
				_subjectModel.getTendonLength ( data );
				_mapLogIDToNumerOfRow[TendonLength]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;

			case LMT:
				_subjectModel.getMuscleTendonLengths ( data );
				_mapLogIDToNumerOfRow[LMT]++;
				fillData ( _mapLogIDToVect.at ( logID ), time, data );
				break;
		}
	else
		COUT << "NMSModel not set for logging." << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time, const std::vector<double>& data )
{
	using namespace Logger;
// 	std::ofstream* file = _mapLogIDToFile.at ( logID );

	switch ( logID )
	{
		case MotorTorque:
			_mapLogIDToNumerOfRow[MotorTorque]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;
			
		case FootSwitch:
			_mapLogIDToNumerOfRow[FootSwitch]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;
				
		case Emgs:
			_mapLogIDToNumerOfRow[Emgs]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;
			
		case TorquesFilt:
			_mapLogIDToNumerOfRow[TorquesFilt]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case EmgsFilter:
			_mapLogIDToNumerOfRow[EmgsFilter]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case LMT:
			_mapLogIDToNumerOfRow[LMT]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case ForcePlate:
			_mapLogIDToNumerOfRow[ForcePlate]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case ForcePlateFilter:
			_mapLogIDToNumerOfRow[ForcePlateFilter]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case IK:
			_mapLogIDToNumerOfRow[IK]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case ID:
			_mapLogIDToNumerOfRow[ID]++;
			fillData ( _mapLogIDToVect.at ( logID ), time, data );
			break;

		case Marker:
// 			_mapLogIDToNumerOfRow[Marker]++;
// 			*file << _cptMarker << "\t" << std::setprecision ( 15 ) << time << "\t";
// 
// 			for ( std::vector<double>::const_iterator it = data.begin(); it != data.end(); it++ )
// 				*file << std::setprecision ( 15 ) << *it << "\t";
// 
// 			*file << std::endl;
// 			_cptMarker ++;
			break;

		case MarkerFilter:
// 			_mapLogIDToNumerOfRow[MarkerFilter]++;
// 			*file << _cptMarkerFilter << "\t" << std::setprecision ( 15 ) << time << "\t";
// 
// 			for ( std::vector<double>::const_iterator it = data.begin(); it != data.end(); it++ )
// 				*file << std::setprecision ( 15 ) << *it << "\t";
// 
// 			*file << std::endl;
// 			_cptMarkerFilter++;
			break;
	}
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time, const double& data )
{
	using namespace Logger;
// 	COUT << time << std::endl;
// 	std::ofstream* file = _mapLogIDToFile.at ( logID );
/*
	if ( logID != NMSTiming && logID != TotalTiming && logID != IKTiming && logID != MTUTiming && logID != Battery )
		_mapLogIDToNumerOfRow[logID]++;

	*file << std::setprecision ( 15 ) << time << "\t" << data << std::endl;*/
       std::vector<double> temp;
       temp.push_back(time);
       temp.push_back(data);
       _mapLogIDToVect.at ( logID ).push_back(temp);
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time, const std::vector<bool>& data )
{
// 	using namespace Logger;
// 	std::ofstream* file = _mapLogIDToFile.at ( logID );
// 	
// 	switch ( logID )
// 	{
// 			
// 		case FootSwitch:
// 			_mapLogIDToNumerOfRow[FootSwitch]++;
// 			fillData ( *file, time, data );
// 			break;
// 	}
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::fillData ( std::ofstream& file, const double& time, const std::vector<double>& data )
{
	file << std::setprecision ( 15 ) << time << "\t";

	for ( std::vector<double>::const_iterator it = data.begin(); it != data.end(); it++ )
		file << std::setprecision ( 15 ) << *it << "\t";

	file << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::fillData ( std::ofstream& file, const double& time, const std::vector<bool>& data )
{
	file << std::setprecision ( 15 ) << time << "\t";

	for ( std::vector<bool>::const_iterator it = data.begin(); it != data.end(); it++ )
		file << std::setprecision ( 15 ) << *it << "\t";

	file << std::endl;
}


template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::fillData(std::vector<std::vector<double> >& vectData, const double& time, const std::vector<double>& data)
{
    std::vector<double> temp;
    temp.push_back(time);
    temp.insert(temp.end(), data.begin(), data.end());
    vectData.push_back(temp);
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::logMa ( const std::vector<std::string>& maNameList, const double& time, const std::vector<std::vector<double> >& data )
{
	for ( std::vector<std::string>::const_iterator it = maNameList.begin(); it != maNameList.end(); it++ )
	{
		int cpt = std::distance<std::vector<std::string>::const_iterator> ( maNameList.begin(), it );
		logMa ( *it, time, data.at ( cpt ) );
	}
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::logMa ( const std::string& maName, const double& time, const std::vector<double>& data )
{
	_mapMANametoNumerOfRow[maName]++;
	fillData ( _mapMANametoVect.at ( maName ), time, data );
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::stop()
{
	using namespace Logger;
	
	COUT << "Saving to File...." << std::flush;
	for ( std::map<Logger::LogID, std::ofstream* >::iterator it = _mapLogIDToFile.begin(); it != _mapLogIDToFile.end(); it++ )
	{
		if ( it->first != NMSTiming && it->first != TotalTiming && it->first != IKTiming && it->first != MTUTiming && it->first != Battery )
		{
			std::string header;
			std::stringstream ss, ssCopy;
			ss << "./";
			ss << _recordDirectory;
			ssCopy << "./";
			ssCopy << _recordDirectory;

			switch ( it->first )
			{
				case Activations:
					header = "Activations";
					ss << "/Activations.sto";
					ssCopy << "/Activations_copy.sto";
					break;
					
				case FootSwitch:
					header = "FootSwitch";
					ss << "/FootSwitch.sto";
					ssCopy << "/FootSwitch_copy.sto";
					break;
					
				case MotorTorque:
					header = "MotorTorque";
					ss << "/MotorTorque.sto";
					ssCopy << "/MotorTorque_copy.sto";
					break;

				case FibreLengths:
					header = "FibreLengths";
					ss << "/FibreLengths.sto";
					ssCopy << "/FibreLengths_copy.sto";
					break;

				case FibreVelocities:
					header = "FibreVelocities";
					ss << "/FibreVelocities.sto";
					ssCopy << "/FibreVelocities_copy.sto";
					break;

				case MuscleForces:
					header = "MuscleForces";
					ss << "/MuscleForces.sto";
					ssCopy << "/MuscleForces_copy.sto";
					break;

				case Torques:
					header = "Torques";
					ss << "/Torque.sto";
					ssCopy << "/Torques_copy.sto";
					break;
					
				case TorquesFilt:
					header = "TorquesFilt";
					ss << "/TorquesFilt.sto";
					ssCopy << "/TorquesFilt_copy.sto";
					break;

				case Logger::PennationAngle:
					header = "PennationAngle";
					ss << "/PennationAngle.sto";
					ssCopy << "/PennationAngle_copy.sto";
					break;

				case TendonLength:
					header = "TendonLength";
					ss << "/TendonLength.sto";
					ssCopy << "/TendonLength_copy.sto";
					break;

				case ForcePlate:
					header = "forcePlate";
					ss << "/forcePlate.sto";
					ssCopy << "/forcePlate_copy.sto";
					break;

				case ForcePlateFilter:
					header = "forcePlateFilter";
					ss << "/forcePlateFilt.sto";
					ssCopy << "/forcePlateFilt_copy.sto";
					break;

				case LMT:
					header = "lmt";
					ss << "/lmt.sto";
					ssCopy << "/lmt_copy.sto";
					break;
					
				case Emgs:
					header = "Emgs";
					ss << "/emg.sto";
					ssCopy << "/emg_copy.sto";
					break;
					
				case EmgsFilter:
					header = "EmgsFilter";
					ss << "/emgFilt.sto";
					ssCopy << "/emgFilt_copy.sto";
					break;

				case IK:
					header = "IK";
					ss << "/ik.sto";
					ssCopy << "/ik_copy.sto";
					break;

				case ID:
					header = "ID";
					ss << "/id.sto";
					ssCopy << "/id_copy.sto";
					break;

				case Marker:
					ss << "/marker.trc";
					ssCopy << "/marker_copy.trc";
					break;

				case MarkerFilter:
					ss << "/markerFilt.trc";
					ssCopy << "/MarkerFilter_copy.trc";
					break;
			}

			std::rename ( ss.str().c_str(), ssCopy.str().c_str() );
			it->second->close();
			std::ifstream infile ( ssCopy.str().c_str() );
			std::ofstream outfile ( ss.str().c_str() );

			if ( it->first != MarkerFilter && it->first != Marker )
			{
				HeaderFile headerFile;
				headerFile.readFile ( infile, ssCopy.str().c_str() );
				headerFile.setNumberOfRow ( _mapLogIDToNumerOfRow[it->first] );
				headerFile.writeFile ( outfile, ss.str(), header );
			}
// 			else if ( it->first == MarkerFilter || it->first == Marker )
// 			{
// 				outfile << "PathFileType\t4\t(X/Y/Z)\tmarker.trc" << std::endl;
// 				markerHearder ( outfile, columnMarkerNames_, _mapLogIDToNumerOfRow[it->first] );
// 				std::string line;
// 				std::getline ( infile, line, '\n' );
// 				std::getline ( infile, line, '\n' );
// 				std::getline ( infile, line, '\n' );
// 				std::getline ( infile, line, '\n' );
// 				std::getline ( infile, line, '\n' );
// 				std::getline ( infile, line, '\n' );
// 			}

			for(std::vector<std::vector<double> >::const_iterator it2 = _mapLogIDToVect.at(it->first).begin(); it2 != _mapLogIDToVect.at(it->first).end(); it2++)
			{
				for(std::vector<double>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++)
				{
					outfile << std::setprecision ( 15 ) << *it3 << "\t";
				}
				outfile << std::endl;
			}
// 			outfile << infile.rdbuf();
			outfile.close();
			infile.close();
			std::remove(ssCopy.str().c_str());
		}
		else
		{
		  
		  for(std::vector<std::vector<double> >::const_iterator it2 = _mapLogIDToVect.at(it->first).begin(); it2 != _mapLogIDToVect.at(it->first).end(); it2++)
			{
				for(std::vector<double>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++)
				{
					*(it->second) << std::setprecision ( 15 ) << *it3 << "\t";
				}
				*(it->second) << std::endl;
			}
		}
	}

	for(std::map<std::string, std::ofstream*>::iterator it = _mapMANametoFile.begin(); it != _mapMANametoFile.end(); it++)
	{
		std::stringstream ss, ssCopy;
		ss << "./";
		ss << _recordDirectory;
		ss << "/ma_" << it->first << ".sto";
		ssCopy << "./";
		ssCopy << _recordDirectory;
		ssCopy << "/ma_" << it->first << "_copy.sto";
		it->second->close();
		std::rename ( ss.str().c_str(), ssCopy.str().c_str() );
		std::ifstream infile ( ssCopy.str().c_str() );
		std::ofstream outfile ( ss.str().c_str() );
		HeaderFile headerFile;
		headerFile.readFile ( infile, ssCopy.str().c_str() );
		headerFile.setNumberOfRow ( _mapMANametoNumerOfRow[it->first] );
		headerFile.writeFile ( outfile, ss.str(), it->first );
// 		outfile << infile.rdbuf();
		for(std::vector<std::vector<double> >::const_iterator it2 = _mapMANametoVect.at(it->first).begin(); it2 != _mapMANametoVect.at(it->first).end(); it2++)
			{
				for(std::vector<double>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++)
				{
					outfile << std::setprecision ( 15 ) << *it3 << "\t";
				}
				outfile << std::endl;
			}
		outfile.close();
		infile.close();
		std::remove(ssCopy.str().c_str());
	}
	
	COUT << "END" << std::endl << std::flush;
}
