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

#include <vector>
using std::vector;
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::endl;
#include <sstream>
using std::stringstream;
#include <stdlib.h>
#include "DataFromFile.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <memory>
#include <limits>

DataFromFile::DataFromFile( const string& dataFileName )
	: dataFile_ ( dataFileName.c_str() ), dataFileName_ ( dataFileName ), noTimeSteps_ ( 0 ), _currentDataTime ( -1 ), _nextDataTime (-1), currentTimeStep_ ( 0 ), _estimatedSampleTime{0}
{
	this->_bufferedDataAvailable = true;
	if ( !dataFile_.is_open() )
	{
		COUT << "ERROR: " << dataFileName_ << " could not be open\n";
		exit ( EXIT_FAILURE );
	}
	this->_fileType = dataFileName.substr(dataFileName.find_last_of(".") + 1);

	if(this->_fileType == "sto")
		this->headerFile_ = std::make_shared<HeaderFile>();
	else if(this->_fileType == "mot")
		this->headerFile_ = std::make_shared<MotHeaderFile>();
	else
		throw std::runtime_error("Data file type " + this->_fileType + " parsing not implemented");

	// this->headerFile_->readFile ( dataFile_, dataFileName_ );
	// _currentData.resize ( headerFile_->getNumberOfColumn() - 1 ); // Minus one for the time
	this->resetFile();

}

void DataFromFile::resetFile(void){
	this->dataFile_.clear();
	this->dataFile_.seekg(0);
	this->noTimeSteps_ = 0 ; // This variable seems unused. Kill it after revision
	this->_currentDataTime = -1;
	this->_nextDataTime = -1;
	this->currentTimeStep_ = 0 ;
	this->_estimatedSampleTime = 0;
	this->_currentData.clear();
	this->_nextData.clear();


	this->headerFile_->readFile ( dataFile_, dataFileName_ ); // Skips header
	// _currentData.resize ( headerFile_->getNumberOfColumn() - 1 ); // Minus one for the time

	// this->readNextData(); // Sets up first data

}


void DataFromFile::readNextData(void){
	this->readNextData(std::numeric_limits<double>::lowest());
}


void DataFromFile::readNextData(double skipToTime) // Default value of -infinity for parameter
{
	bool parseData = true;
	while(skipToTime >= this->_nextDataTime || skipToTime == std::numeric_limits<double>::lowest()){
		this->_currentData = this->_nextData;
		this->_currentDataTime = this->_nextDataTime;

		if(!this->_bufferedDataAvailable){
			currentTimeStep_++;
			return; // No more data available. 
		}

		// read time for the data currently stored in DataFromFile
		string line;
		currentTimeStep_++;
		getline ( dataFile_, line, '\n' );
		line.erase(remove(line.begin(), line.end(), ' '), line.end());
		stringstream myStream ( line );
		double value;
		std::string item;
		this->_nextData.clear();

		std::getline(myStream, item, '\t');
		this->_nextDataTime = std::stod(item);
		parseData = true;
		if(this->_nextDataTime < skipToTime && 	skipToTime != std::numeric_limits<double>::lowest()){ // if skipToTime is default, just step blindly
			parseData = false;
			this->_nextData.clear(); // Invalidates next data, as parsing will not occurr. 
			if(this->_estimatedSampleTime){ // Sample time is already estimated
				int linesToIgnore = (int)((skipToTime - this->_nextDataTime) / this->_estimatedSampleTime) - 1; // Rounded down in case of non exact division
				for (int linesIgnored = 0; linesIgnored < linesToIgnore; ++linesIgnored){
					this->dataFile_.ignore(std::numeric_limits<std::streamsize>::max(), this->dataFile_.widen('\n'));
				}
				currentTimeStep_ += linesToIgnore;
			}
		}

		while (std::getline(myStream, item, '\t') && parseData) { 	// Shaves off a few microseconds every iteration not to parse the data if it is to be skipped.
																	// For high sample rate data (ex 1000Hz), it makes a massive difference
			value = nan("NAN");
			value = std::stod(item);
			if(!std::isnan(value))
			{
				if ( headerFile_->getInDegrees() )
					value = value / 180 * M_PI;

				this->_nextData.push_back ( value );
			}
		}

		if ( (this->_nextData.size() != headerFile_->getNumberOfColumn() - 1) && parseData)
		{

		stringstream myStream ( line );
		double value;
		std::string item;
		this->_nextData.clear();

		std::getline(myStream, item, '\t');
		_currentDataTime = std::stod(item);

		while (std::getline(myStream, item, '\t')) { 	
																	
			value = nan("NAN");
			value = std::stod(item);
			if(!std::isnan(value))
			{
				if ( headerFile_->getInDegrees() )
					value = value / 180 * M_PI;

				this->_nextData.push_back ( value );
			}
		}

			COUT << "ERROR: in " << dataFileName_ << " at time step " << currentTimeStep_ << " you have " << this->_nextData.size() << " input.\nYou need " << headerFile_->getNumberOfColumn() - 1 << endl << std::flush;
			exit ( EXIT_FAILURE );
		}

		if(_currentData.size() == 0 && parseData) // The first iteration sets both the current and next data
			this->readNextData();
		if(this->_nextDataTime > 0 && this->_currentDataTime > 0) // Assumes that time cannot be negative. Could break
			this->_estimatedSampleTime = this->_nextDataTime - this->_currentDataTime;


		if(!this->dataAvailableOnFile()){
			this->_bufferedDataAvailable = false;
		}

		if(skipToTime == std::numeric_limits<double>::lowest())
			break;

	}
}

DataFromFile::~DataFromFile()
{
	dataFile_.close();
}

// This funciton is extremely inefficient. 
void DataFromFile::stepToTime(double timeStamp){
	while((timeStamp > this->_currentDataTime) && !(timeStamp >= this->_currentDataTime && timeStamp < this->_nextDataTime) && this->dataAvailableOnFile()){
	this->readNextData(timeStamp);
	}
}
