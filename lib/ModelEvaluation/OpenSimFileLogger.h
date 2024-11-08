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

#ifndef OPENSIMFILELOGGER_H
#define OPENSIMFILELOGGER_H

#include "CommonCEINMS.h"
#include <HeaderFile.h>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <map>
#include <deque>

#include <thread>
#include <mutex>
#include <deque>

// Handle difference between sleep() and Sleep() between Windows and Linux
#ifndef _WIN32
// In case the Windows header wasn't loaded, create an alias
#define Sleep sleep
#endif

namespace Logger
{
	enum LogID
	{
		Activations,
		FibreLengths,
		FibreVelocities,
		MuscleForces,
		MusclePassiveForces,
		MuscleActiveForces,
		TendonStrain,
		LMT,
		Torques,
		TorquesFilt,
		PennationAngle,
		TendonLength,
		MA,
		Emgs,
		EmgsFilter,
		Marker,
		MarkerFilter,
		ForcePlate,
		ForcePlateFilter,
		IK,
		ID,
		NMSTiming,
		TotalTiming,
		IKTiming,
		MTUTiming,
		RandomSignal,
		ShapeFactor,
		TendonSlackLengths,
		OptimalFiberLengths,
		GroupMusclesBasedOnStrengthCoefficients,
		Error,
		OptimizationParameter,
		IKXSENS
	};

	struct loggerStruct
	{
		std::ofstream* file;
		double time;
		std::vector<double> data;
	};
};

template <typename NMSmodelT>
class OpenSimFileLogger
{
	public:

		OpenSimFileLogger ( const NMSmodelT& subjectModel, const std::string& recordDirectory );
		OpenSimFileLogger ( const std::string& recordDirectory );
		~OpenSimFileLogger();

		void addLog ( Logger::LogID logID, const std::vector<std::string>& ColumnName );
		void addLog ( Logger::LogID logID);
		void addLogCSV (std::string filename);
		void addLogCSVwithHeader(std::string filename, const std::vector<std::string>& ColumnName);
		void addMa ( const std::string& maName, const std::vector<std::string>& ColumnName );
		void log ( Logger::LogID logID, const double& time );
		void logCSV (std::string filename, const double& time, const std::vector<double>& data);
		void logCSV(std::string filename, const double& time, const double& data);
		void log ( Logger::LogID logID, const double& time, const std::vector<double>& data );
		// void log ( Logger::LogID logID, const double& time, const std::vector<bool>& data );
		void log ( Logger::LogID logID, const double& time, const double& data );
		void logMa(const std::vector<std::string>& maNameList, const double& time, const std::vector<std::vector<double> >& data );
		void logMa(const std::string& maName, const double& time, const std::vector<double>& data );
		void stop();
		
	private:

		std::string _recordDirectory;
		bool _subjectModelGiven;
		const NMSmodelT& _subjectModel;
		unsigned int _cptMarker;
		unsigned int _cptMarkerFilter;
		std::vector<std::string> columnMarkerNames_;
		std::map<Logger::LogID, std::ofstream*> _mapLogIDToFile;
		std::map<Logger::LogID, std::vector<std::vector<double> > > _mapLogIDToVect;
		std::map<Logger::LogID, unsigned int> _mapLogIDToNumerOfRow;
		std::map<std::string, std::ofstream*> _mapMANametoFile;
		std::map<std::string,  std::vector<std::vector<double> > > _mapMANametoVect;
		std::map<std::string, unsigned int> _mapMANametoNumerOfRow;
		std::map<std::string, std::ofstream*> _mapLogIDCSVToFile;

		std::thread *saveThread_;
		std::mutex mtxdata_;
		std::mutex mtxEnd_;
		bool threadStop_;
		std::deque<Logger::loggerStruct> loggerStructDeque_;
		
		void threadFunc();
		
		void markerHearder ( std::ofstream& FilePtr, const std::vector<std::string>& ColumnName, const unsigned int& numbersOfFrames );
		void fillData(std::ofstream& FilePtr, const double& time, const std::vector<double>& data);
		void fillData(std::vector<std::vector<double> >& vectData, const double& time, const std::vector<double>& data);
		void fillData(std::ofstream& FilePtr, const double& time, const std::vector<bool>& data);

};

#ifdef RPI
  #include "OpenSimFileLoggerRPI.cpp"
#else
  #include "OpenSimFileLogger.cpp"
#endif

#endif // OPENSIMFILELOGGER_H
