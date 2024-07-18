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

// #include "OpenSimFileLogger.h"

template <typename NMSmodelT>
OpenSimFileLogger<NMSmodelT>::OpenSimFileLogger ( const NMSmodelT& subjectModel, const std::string& recordDirectory ) :
_subjectModel(subjectModel), _subjectModelGiven(true), _recordDirectory(recordDirectory), _cptMarker(0), _cptMarkerFilter(0), threadStop_(false)
{
	// Create the directory for the recorded file
	std::stringstream ss;
	ss << "./";
	ss << _recordDirectory;
	boost::filesystem::path dir ( ss.str().c_str() );

	if ( !boost::filesystem::exists ( dir ) )
		if ( !boost::filesystem::create_directories ( dir ) )
			COUT << "ERROR in creating directory: " << ss.str() << std::endl;

	saveThread_ = new std::thread(&OpenSimFileLogger<NMSmodelT>::threadFunc, this);
}

template <typename NMSmodelT>
OpenSimFileLogger<NMSmodelT>::OpenSimFileLogger ( const std::string& recordDirectory ) :
_subjectModelGiven(false), _recordDirectory(recordDirectory), _cptMarker(0), _cptMarkerFilter(0), _subjectModel(NMSmodelT()), threadStop_(false)
{
	// Create the directory for the recorded file
	std::stringstream ss;
	ss << "./";
	ss << _recordDirectory;
	boost::filesystem::path dir ( ss.str().c_str() );

	if ( !boost::filesystem::exists ( dir ) )
		if ( !boost::filesystem::create_directories ( dir ) )
			COUT << "ERROR in creating directory: " << ss.str() << std::endl;

	saveThread_ = new std::thread(&OpenSimFileLogger<NMSmodelT>::threadFunc, this);
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
void OpenSimFileLogger<NMSmodelT>::threadFunc()
{

	mtxEnd_.lock();
	bool threadStop = threadStop_;
	mtxEnd_.unlock();

	while (!threadStop || loggerStructDeque_.size() > 0)
	{
		bool empty = true;
		Logger::loggerStruct temp;
		mtxdata_.lock();
		if (loggerStructDeque_.size() > 0)
		{
			temp = loggerStructDeque_.front();
			loggerStructDeque_.pop_front();
			empty = false;
		}
		mtxdata_.unlock();

		if (!empty)
		{
			*temp.file << std::setprecision(15) << temp.time << "\t";

			for (std::vector<double>::const_iterator it = temp.data.begin(); it != temp.data.end(); it++)
				*temp.file << std::setprecision(15) << *it << "\t";

			*temp.file << std::endl;
		}
		else
		{
		#ifdef _win_
			Sleep(300);//Waiting for more data
			#else 
				#ifdef UNIX
					usleep(300);
				#endif
			#endif
		}

		mtxEnd_.lock();
		threadStop = threadStop_;
		mtxEnd_.unlock();

	}
	std::cout << "logger Size"<< loggerStructDeque_.size() << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::addLogCSV (std::string filename)
{
	using namespace Logger;
	std::stringstream ss;

	ss << "./";
	ss << _recordDirectory;
	ss << "/";
	ss << filename;
	ss << ".csv";
	_mapLogIDCSVToFile[filename] = new std::ofstream(ss.str().c_str());

	if (!(_mapLogIDCSVToFile[filename]->is_open()))
		COUT << "ERROR: " + ss.str() + " cannot be opened!" << std::endl;

}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::addLogCSVwithHeader(std::string filename, const std::vector<std::string>& ColumnName)
{
	using namespace Logger;
	std::stringstream ss;

	// create full saving path
	ss << "./";
	ss << _recordDirectory;
	ss << "/";
	ss << filename;
	ss << ".csv";

	// open a new csv file
	_mapLogIDCSVToFile[filename] = new std::ofstream(ss.str().c_str());

	// create file variable for better code later on
	std::ofstream* file = _mapLogIDCSVToFile[filename];

	// check if the csv file can be opened
	if (!(file->is_open()))
		COUT << "ERROR: " + ss.str() + " cannot be opened!" << std::endl;

	*file << "Time\t";  // saving the Time as the first header variables

	for (int i = 0; i < ColumnName.size(); ++i) // saving all header strings
	{
		*file << ColumnName.at(i) << "\t";
	}

	*file << "\n";  // close the line

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

		case FibreLengths:
			filename = "/FibreLengths.sto";
			_mapLogIDToNumerOfRow[FibreLengths] = 0;
			break;		

		case FibreVelocities:
			filename = "/FibreVelocities.sto";
			_mapLogIDToNumerOfRow[FibreVelocities] = 0;
			break;

		case MuscleForces:
			filename = "/MuscleForces.sto";
			_mapLogIDToNumerOfRow[MuscleForces] = 0;
			break;

		case MusclePassiveForces:
			filename = "/MusclePassiveForces.sto";
			_mapLogIDToNumerOfRow[MusclePassiveForces] = 0;
			break;

		case MuscleActiveForces:
			filename = "/MuscleActiveForces.sto";
			_mapLogIDToNumerOfRow[MuscleActiveForces] = 0;
			break;

		case TendonStrain:
			filename = "/TendonStrain.sto";
			_mapLogIDToNumerOfRow[TendonStrain] = 0;
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
			filename = "/forcePlate.mot";
			_mapLogIDToNumerOfRow[ForcePlate] = 0;
			break;

		case ForcePlateFilter:
			filename = "/forcePlateFilt.mot";
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

		case ShapeFactor:
			filename = "/ShapeFactor.sto";
			_mapLogIDToNumerOfRow[ShapeFactor] = 0;
			break;

		case TendonSlackLengths:
			filename = "/TendonSlackLengths.sto";
			_mapLogIDToNumerOfRow[TendonSlackLengths] = 0;
			break;

		case OptimalFiberLengths:
			filename = "/OptimalFiberLengths.sto";
			_mapLogIDToNumerOfRow[OptimalFiberLengths] = 0;
			break;

		case GroupMusclesBasedOnStrengthCoefficients:
			filename = "/GroupMusclesBasedOnStrengthCoefficients.sto";
			_mapLogIDToNumerOfRow[GroupMusclesBasedOnStrengthCoefficients] = 0;
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

		case Error:
			filename = "/Error.csv";
			break;

		case OptimizationParameter:
			filename = "/OptimizationParameter.csv";
			break;
		case IKXSENS:
			filename = "/ikXsens.sto";
			_mapLogIDToNumerOfRow[IKXSENS] = 0;
			break;
	}

	ss << "./";
	ss << _recordDirectory;
	ss << filename;
	_mapLogIDToFile[logID] = new std::ofstream ( ss.str().c_str() );
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
			
		case FibreLengths:
			headerFile.writeFile ( *file, ss.str(), "FibreLength" );
			break;

		case FibreVelocities:
			headerFile.writeFile ( *file, ss.str(), "FibreVelocities" );
			break;

		case MuscleForces:
			headerFile.writeFile ( *file, ss.str(), "MuscleForces" );
			break;

		case MuscleActiveForces:
			headerFile.writeFile(*file, ss.str(), "MuscleActiveForces");
			break;

		case MusclePassiveForces:
			headerFile.writeFile(*file, ss.str(), "MusclePassiveForces");
			break;

		case TendonStrain:
			headerFile.writeFile(*file, ss.str(), "TendonStrain");
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

		case IKXSENS:
			headerFile.writeFile(*file, ss.str(), "IKXsens");
			break;

		case ID:
			headerFile.writeFile ( *file, ss.str(), "ID" );
			break;

		case ShapeFactor:
			headerFile.writeFile(*file, ss.str(), "ShapeFactor");
			break;

		case TendonSlackLengths:
			headerFile.writeFile(*file, ss.str(), "TendonSlackLengths");
			break;

		case OptimalFiberLengths:
			headerFile.writeFile(*file, ss.str(), "OptimalFiberLengths");
			break;

		case GroupMusclesBasedOnStrengthCoefficients:
			headerFile.writeFile(*file, ss.str(), "GroupMusclesBasedOnStrengthCoefficients");
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

		case RandomSignal:
			filename = "/RandomSignal.csv";
			break;

		case Error:
			filename = "/Error.csv";
			break;

		case OptimizationParameter:
			filename = "/OptimizationParameter.csv";
			break;

		default:
			COUT << "Error wong addlog method for " << logID << std::endl;
			break;
	}

	ss << "./";
	ss << _recordDirectory;
	ss << filename;
	_mapLogIDToFile[logID] = new std::ofstream ( ss.str().c_str() );

	if ( ! ( _mapLogIDToFile[logID]->is_open() ) )
		COUT << "ERROR: " + ss.str() + " cannot be opened!" << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::markerHearder ( std::ofstream& file, const std::vector<std::string>& ColumnName, const unsigned int& numbersOfFrames )
{
	file << "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames" << std::endl;
	file << "128\t128\t" << numbersOfFrames << "\t" << ColumnName.size() << "\tm\t128\t1\t" << numbersOfFrames << std::endl;
	file << "Frame#\tTime\t";

	for ( std::vector<std::string>::const_iterator it = ColumnName.begin(); it != ColumnName.end(); it++ )
		file << *it << "\t";

	file << std::endl;
	file << "\t\t";

	for ( int cpt = 1; cpt < ColumnName.size()+1; cpt++ )
	{
		file << "X" << cpt << "\t";
		file << "Y" << cpt << "\t";
		file << "Z" << cpt << "\t";
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
	headerFile.setNumberOfRow ( 0 );
	headerFile.setNumberOfColumn ( ColumnName.size() + 1 ); // +1 for time
	headerFile.setNameOfColumn ( ColumnName );
	headerFile.setInDegrees ( false );
	headerFile.writeFile ( *file, "ma_" + maName + ".sto", "ma_" + maName );
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::logCSV(std::string filename, const double& time, const std::vector<double>& data)
{
	using namespace Logger;
	std::ofstream* file = _mapLogIDCSVToFile.at(filename);
	fillData(*file, time, data);
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::logCSV(std::string filename, const double& time, const double& data)
{
	using namespace Logger;
	std::ofstream* file = _mapLogIDCSVToFile.at(filename);
	std::vector<double> dataVectTemp;
	dataVectTemp.push_back(data);
	fillData(*file, time, dataVectTemp);
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time )
{
	using namespace Logger;
	std::ofstream* file = _mapLogIDToFile.at ( logID );
	std::vector<double> data;

	if ( _subjectModelGiven )
		switch ( logID )
		{
			case Activations:
				_subjectModel.getActivations ( data );
				_mapLogIDToNumerOfRow[Activations]++;
				fillData ( *file, time, data );
				break;

			case FibreLengths:
				_subjectModel.getFiberLengths ( data );
				_mapLogIDToNumerOfRow[FibreLengths]++;
				fillData ( *file, time, data );
				break;

			case FibreVelocities:
				_subjectModel.getFiberVelocities ( data );
				_mapLogIDToNumerOfRow[FibreVelocities]++;
				fillData ( *file, time, data );
				break;

			case MuscleForces:
				_subjectModel.getMuscleForces ( data );
				_mapLogIDToNumerOfRow[MuscleForces]++;
				fillData ( *file, time, data );
				break;

			case MusclePassiveForces:
				_subjectModel.getMusclePassiveForces(data);
				_mapLogIDToNumerOfRow[MusclePassiveForces]++;
				fillData(*file, time, data);
				break;

			case MuscleActiveForces:
				_subjectModel.getMuscleActiveForces(data);
				_mapLogIDToNumerOfRow[MuscleActiveForces]++;
				fillData(*file, time, data);
				break;

			case TendonStrain:
				_subjectModel.getTendonStrain(data);
				_mapLogIDToNumerOfRow[TendonStrain]++;
				fillData(*file, time, data);
				break;

			case Torques:
				_subjectModel.getTorques ( data );
				_mapLogIDToNumerOfRow[Torques]++;
				fillData ( *file, time, data );
				break;

			case Logger::PennationAngle:
				_subjectModel.getPennationAngleInst ( data );
				_mapLogIDToNumerOfRow[Logger::PennationAngle]++;
				fillData ( *file, time, data );
				break;

			case TendonLength:
				_subjectModel.getTendonLength ( data );
				_mapLogIDToNumerOfRow[TendonLength]++;
				fillData ( *file, time, data );
				break;

			case LMT:
				_subjectModel.getMuscleTendonLengths ( data );
				_mapLogIDToNumerOfRow[LMT]++;
				fillData ( *file, time, data );
				break;

			case ShapeFactor:
				_subjectModel.getShapeFactors(data);
				_mapLogIDToNumerOfRow[ShapeFactor]++;
				fillData(*file, time, data);
				break;

			case TendonSlackLengths:
				_subjectModel.getTendonSlackLengths(data);
				_mapLogIDToNumerOfRow[TendonSlackLengths]++;
				fillData(*file, time, data);
				break;

			case OptimalFiberLengths:
				_subjectModel.getOptimalFiberLengths(data);
				_mapLogIDToNumerOfRow[OptimalFiberLengths]++;
				fillData(*file, time, data);
				break;

			case Emgs:
				_subjectModel.getEmgs(data);
				_mapLogIDToNumerOfRow[Emgs]++;
				fillData(*file, time, data);
				break;

			case GroupMusclesBasedOnStrengthCoefficients:
				std::vector< std::vector< int > > temp;
				_subjectModel.getGroupMusclesBasedOnStrengthCoefficients(data, temp);
				_mapLogIDToNumerOfRow[GroupMusclesBasedOnStrengthCoefficients]++;
				fillData(*file, time, data);
				break;
		}
	else
		COUT << "NMSModel not set for logging." << std::endl;
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time, const std::vector<double>& data )
{
	using namespace Logger;
	std::ofstream* file = _mapLogIDToFile.at ( logID );

	switch ( logID )
	{
		//case MotorTorque:
		//	_mapLogIDToNumerOfRow[MotorTorque]++;
		//	fillData ( *file, time, data );
		//	break;
			
		//case FootSwitch:
		//	_mapLogIDToNumerOfRow[FootSwitch]++;
		//	fillData ( *file, time, data );
		//	break;
				
		case Emgs:
			_mapLogIDToNumerOfRow[Emgs]++;
			fillData ( *file, time, data );
			break;
			
		case TorquesFilt:
			_mapLogIDToNumerOfRow[TorquesFilt]++;
			fillData ( *file, time, data );
			break;

		case EmgsFilter:
			_mapLogIDToNumerOfRow[EmgsFilter]++;
			fillData ( *file, time, data );
			break;

		case LMT:
			_mapLogIDToNumerOfRow[LMT]++;
			fillData ( *file, time, data );
			break;

		case ForcePlate:
			_mapLogIDToNumerOfRow[ForcePlate]++;
			fillData ( *file, time, data );
			break;

		case ForcePlateFilter:
			_mapLogIDToNumerOfRow[ForcePlateFilter]++;
			fillData ( *file, time, data );
			break;

		case IK:
			_mapLogIDToNumerOfRow[IK]++;
			fillData ( *file, time, data );
			break;

		case IKXSENS:
			_mapLogIDToNumerOfRow[IKXSENS]++;
			fillData(*file, time, data);
			break;

		case ID:
			_mapLogIDToNumerOfRow[ID]++;
			fillData ( *file, time, data );
			break;

		case ShapeFactor:
			_mapLogIDToNumerOfRow[ShapeFactor]++;
			fillData(*file, time, data);
			break;

		case TendonSlackLengths:
			_mapLogIDToNumerOfRow[TendonSlackLengths]++;
			fillData(*file, time, data);
			break;

		case OptimalFiberLengths:
			_mapLogIDToNumerOfRow[OptimalFiberLengths]++;
			fillData(*file, time, data);
			break;

		case GroupMusclesBasedOnStrengthCoefficients:
			_mapLogIDToNumerOfRow[GroupMusclesBasedOnStrengthCoefficients]++;
			fillData(*file, time, data);
			break;

		case Marker:
			_mapLogIDToNumerOfRow[Marker]++;
			*file << _cptMarker << "\t" << std::setprecision ( 15 ) << time << "\t";

			for ( std::vector<double>::const_iterator it = data.begin(); it != data.end(); it++ )
				*file << std::setprecision ( 15 ) << *it << "\t";

			*file << std::endl;
			_cptMarker ++;
			break;

		case MarkerFilter:
			_mapLogIDToNumerOfRow[MarkerFilter]++;
			*file << _cptMarkerFilter << "\t" << std::setprecision ( 15 ) << time << "\t";

			for ( std::vector<double>::const_iterator it = data.begin(); it != data.end(); it++ )
				*file << std::setprecision ( 15 ) << *it << "\t";

			*file << std::endl;
			_cptMarkerFilter++;
			break;
	}
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time, const double& data )
{
	using namespace Logger;
// 	COUT << time << std::endl;
	std::ofstream* file = _mapLogIDToFile.at ( logID );

	if (logID != NMSTiming && logID != TotalTiming && logID != IKTiming && logID != MTUTiming && logID != RandomSignal && logID != Error && logID != OptimizationParameter)
		_mapLogIDToNumerOfRow[logID]++;

	*file << std::setprecision ( 15 ) << time << "\t" << data << std::endl;
}

// template <typename NMSmodelT>
// void OpenSimFileLogger<NMSmodelT>::log ( Logger::LogID logID, const double& time, const std::vector<bool>& data )
// {
// 	using namespace Logger;
// 	std::ofstream* file = _mapLogIDToFile.at ( logID );
	
// 	switch ( logID )
// 	{
			
// 		case FootSwitch:
// 			_mapLogIDToNumerOfRow[FootSwitch]++;
// 			fillData ( *file, time, data );
// 			break;
// 	}
// }

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::fillData ( std::ofstream& file, const double& time, const std::vector<double>& data )
{
	Logger::loggerStruct temp;
	temp.file = &file;
	temp.time = time;
	temp.data = data;
	mtxdata_.lock();
	loggerStructDeque_.push_back(temp);
	mtxdata_.unlock();
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
	std::ofstream* file = _mapMANametoFile.at ( maName );
	_mapMANametoNumerOfRow[maName]++;
	fillData ( *file, time, data );
}

template <typename NMSmodelT>
void OpenSimFileLogger<NMSmodelT>::stop()
{

	std::cout << "OpenSimFileLogger<NMSmodelT>::stop() " << this << std::endl;
	using namespace Logger;

	mtxEnd_.lock();
	threadStop_ = true;
	mtxEnd_.unlock();
	saveThread_->join();

	delete saveThread_;

	for ( std::map<Logger::LogID, std::ofstream* >::iterator it = _mapLogIDToFile.begin(); it != _mapLogIDToFile.end(); it++ )
	{
		if (it->first != NMSTiming && it->first != TotalTiming && it->first != IKTiming && it->first != MTUTiming && it->first != RandomSignal && it->first != Error && it->first != OptimizationParameter)
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

				case MusclePassiveForces:
					header = "MusclePassiveForces";
					ss << "/MusclePassiveForces.sto";
					ssCopy << "/MusclePassiveForces_copy.sto";
					break;

				case MuscleActiveForces:
					header = "MuscleActiveForces";
					ss << "/MuscleActiveForces.sto";
					ssCopy << "/MuscleActiveForces_copy.sto";
					break;

				case TendonStrain:
					header = "TendonStrain";
					ss << "/TendonStrain.sto";
					ssCopy << "/TendonStrain_copy.sto";
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
					ss << "/forcePlate.mot";
					ssCopy << "/forcePlate_copy.mot";
					break;

				case ForcePlateFilter:
					header = "forcePlateFilter";
					ss << "/forcePlateFilt.mot";
					ssCopy << "/forcePlateFilt_copy.mot";
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

				case IKXSENS:
					header = "IKXsens";
					ss << "/ikXsens.sto";
					ssCopy << "/ikXsens_copy.sto";
					break;

				case ID:
					header = "ID";
					ss << "/id.sto";
					ssCopy << "/id_copy.sto";
					break;

				case ShapeFactor:
					header = "ShapeFactor";
					ss << "/ShapeFactor.sto";
					ssCopy << "/ShapeFactor_copy.sto";
					break;

				case TendonSlackLengths:
					header = "TendonSlackLengths";
					ss << "/TendonSlackLengths.sto";
					ssCopy << "/TendonSlackLengths_copy.sto";
					break;

				case OptimalFiberLengths:
					header = "OptimalFiberLengths";
					ss << "/OptimalFiberLengths.sto";
					ssCopy << "/OptimalFiberLengths_copy.sto";
					break;

				case GroupMusclesBasedOnStrengthCoefficients:
					header = "GroupMusclesBasedOnStrengthCoefficients";
					ss << "/GroupMusclesBasedOnStrengthCoefficients.sto";
					ssCopy << "/GroupMusclesBasedOnStrengthCoefficients_copy.sto";
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
			it->second->close();
#ifdef WIN32
			if (std::rename(ss.str().c_str(), ssCopy.str().c_str()) == -1)
			{
				boost::filesystem::path full_path(boost::filesystem::current_path());
				std::cout << "Current path is : " << full_path << std::endl;
				std::cout << ss.str().c_str() << std::endl;
				std::cout << " Error: " << strerror(errno) << std::endl;
			}
#else
			std::rename(ss.str().c_str(), ssCopy.str().c_str());
#endif
			
			std::ifstream infile ( ssCopy.str().c_str() );
			std::ofstream outfile ( ss.str().c_str() );

			if ( it->first != MarkerFilter && it->first != Marker )
			{
				HeaderFile headerFile;
				headerFile.readFile ( infile, ssCopy.str().c_str() );
				if (_mapLogIDToNumerOfRow[it->first] > 0)
					headerFile.setNumberOfRow ( _mapLogIDToNumerOfRow[it->first] - 1 );
				else
					headerFile.setNumberOfRow ( 0 );
				headerFile.writeFile ( outfile, ss.str(), header );
			}
			else if ( it->first == MarkerFilter || it->first == Marker )
			{
				outfile << "PathFileType\t4\t(X/Y/Z)\tmarker.trc" << std::endl;
				markerHearder ( outfile, columnMarkerNames_, _mapLogIDToNumerOfRow[it->first] );
				std::string line;
				std::getline ( infile, line, '\n' );
				std::getline ( infile, line, '\n' );
				std::getline ( infile, line, '\n' );
				std::getline ( infile, line, '\n' );
				std::getline ( infile, line, '\n' );
				std::getline ( infile, line, '\n' );
			}

			outfile << infile.rdbuf();
			outfile.close();
			infile.close();
			std::remove(ssCopy.str().c_str());
		}
		else
		{
			it->second->close();
		}
	}

	for (std::map<std::string, std::ofstream* >::iterator it = _mapLogIDCSVToFile.begin(); it != _mapLogIDCSVToFile.end(); it++)
		it->second->close();

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
		outfile << infile.rdbuf();
		outfile.close();
		infile.close();
		std::remove(ssCopy.str().c_str());
	}
}
