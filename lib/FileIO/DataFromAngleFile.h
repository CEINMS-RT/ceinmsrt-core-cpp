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

#ifndef DataFromAngleFile_h
#define DataFromAngleFile_h

#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

/**
 * \brief This class read from a file either lmt or ma data
 * You should have a file that includes 
 * write here the structure of the file
 */
class DataFromAngleFile
{
public:
	DataFromAngleFile()
	{
		std::cout << "You should not be there\n";
	}
	DataFromAngleFile(const std::string& inputFile);
	const std::vector<std::string>& getDofsNames() const
	{
		return dofsNames_;
	}
	void readNextData();
	int getNoTimeSteps() const
	{
		return noTimeSteps_;
	}
	bool areStillData() const
	{
		return currentTimeStep_ < noTimeSteps_;
	}
	inline double getCurrentTime() const
	{
		return currentDataTime_;
	}
	const std::vector<double>& getCurrentData() const;
	~DataFromAngleFile();
private:
	DataFromAngleFile(const DataFromAngleFile& orig) {};
	DataFromAngleFile& operator=(const DataFromAngleFile& orig) {};
	std::string dataFileName_;
	std::ifstream dataFile_;
	unsigned int noDofs_;
	std::vector<std::string> dofsNames_;
	int currentTimeStep_;
	int noTimeSteps_;
	double currentDataTime_;
	std::vector<double> currentData_;
	bool degrees_;
};

#endif
