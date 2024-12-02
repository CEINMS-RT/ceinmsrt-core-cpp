// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software, regulated by the license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots"
//

#ifndef DATA_FROM_FILE_H
#define DATA_FROM_FILE_H

#include "HeaderFile.h"
#include "MotHeaderFile.h"
#include "HeaderFileBase.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <limits>
#include <iomanip>
#include <limits>



/**
 * \brief This class read from a file either lmt or ma data
 * You should have a file that includes
 * write here the structure of the file
 */

class DataFromFile
{
	public:
		DataFromFile() : _estimatedSampleTime{0}
		{
			std::cout << "You should not be there\n";
		}
		DataFromFile ( const std::string& dataFileName ) ;
		~DataFromFile();

		inline const std::vector<std::string>& getColumnNames() const
		{
			return headerFile_->getNameOfColumn();
		}

		void readNextData(double skipToTime);
		void readNextData(void);

		inline const unsigned int& getNoTimeSteps() const
		{
			return headerFile_->getNumberOfRow();
		}

		// The current and next data are buffered to match timestamps, thus one step less is possible when a step is requested.
		inline bool areStillData() const
		{
			return currentTimeStep_ < headerFile_->getNumberOfRow() + 1; // If the header is corrupted, this doesnt work
																			// Header is corrupted every time any plugin from file is used that does not use _process
			// return (this->_nextDataTime != this->_currentDataTime) || (this->_currentDataTime == -1);
		}

		inline const double& getCurrentTime() const
		{
			return _currentDataTime;
		}

		inline const std::vector<double>& getCurrentData() const
		{
			return _currentData;
		}

		void stepToTime(double timeStamp);
		void resetFile(void); // Resets file from the beginning


	protected:

		inline bool dataAvailableOnFile() const
		{
			return currentTimeStep_ < headerFile_->getNumberOfRow();
			// return areStillData();
		}

		DataFromFile( const DataFromFile& orig ) {};
		DataFromFile& operator= ( const DataFromFile& orig ) {};

		std::string dataFileName_;
		std::string _fileType;
		std::ifstream dataFile_;
		unsigned int currentTimeStep_;
		int noTimeSteps_;
		double _currentDataTime, _nextDataTime;
		std::vector<double> _currentData, _nextData;
		bool _bufferedDataAvailable;
		double _estimatedSampleTime;
		std::shared_ptr<HeaderFileBase> headerFile_;
};

	

#endif
