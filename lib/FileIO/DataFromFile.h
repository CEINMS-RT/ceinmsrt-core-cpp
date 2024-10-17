// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox".
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software,
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS.
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621,
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com

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
