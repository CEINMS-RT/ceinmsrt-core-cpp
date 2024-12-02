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

#include "DataFromAngleFile.h"

//DataFromFile::DataFromFile(const DataFromFile& orig)
//:dataFile_(orig.dataFilename_.c_str()) {

//  if (!dataFile_.is_open()) {
//    cout << "ERROR: " << dataFilename << " could not be open\n";
//    exit(EXIT_FAILURE);
//  }
//  // and then you open it because ifstream cannot be used with copy constructor and operator =
//  dataFileName_ = orig.dataFileName_;
//  noMuscles_ = orig.noMuscles_;
//  muscleNames_ = orig.muscleNames_;
//  noTimeSteps_ = orig.noTimeSteps_;
//  currentDataTime_ = orig.currentDataTime_;
//  currentData_ = orig.currentData_;
//  currentTimeStep_ = orig.currentTimeStep_;
//}

DataFromAngleFile::DataFromAngleFile ( const string& dataFilename ) :
	dataFile_ ( dataFilename.c_str() )
{
	if ( !dataFile_.is_open() )
	{
		cout << "ERROR: " << dataFilename << " could not be open\n";
		exit ( EXIT_FAILURE );
	}

	dataFileName_ = dataFilename;

	// reading number of columns/rows
	string trash;
	string s;
	std::string item;
	std::vector<std::string> elems;
// 	std::getline ( dataFile_, s, '\n' );
	std::getline ( dataFile_, s );
	dataFile_ >> s;
// 	std::cout << s << std::endl;
	std::stringstream ss ( s );

	while ( std::getline ( ss, item, '=' ) )
	{
		elems.push_back ( item );
	}

	if ( elems[0] == "version" )
	{
		std::getline ( dataFile_, s );
		dataFile_ >> s;
		std::stringstream ss ( s );
		elems.clear();

		while ( std::getline ( ss, item, '=' ) )
		{
			elems.push_back ( item );
		}
		
//  		std::cout << elems.back().c_str() << " -- " << elems[0] << std::endl;

		if ( elems[0] == "nRows" )
			noTimeSteps_ = atoi ( elems.back().c_str() );
		else if ( elems[0] == "nColumns" )
			noDofs_ = atoi ( elems.back().c_str() ) - 1;
// 		std::cout << "1: " << noTimeSteps_ << std::endl;
	}
	else
	{
		if ( elems[0] == "nRows" )
			noTimeSteps_ = atoi ( elems.back().c_str() );
		else if ( elems[0] == "nColumns" )
			noDofs_ = atoi ( elems.back().c_str() ) - 1;
// 		std::cout << "2: " << noTimeSteps_ << std::endl;
	}

	dataFile_ >> s;
// 	std::cout << s << std::endl;
	std::stringstream ss2 ( s );
	elems.clear();

	while ( std::getline ( ss2, item, '=' ) )
	{
		elems.push_back ( item );
	}

	if ( elems[0] == "nRows" )
		noTimeSteps_ = atoi ( elems.back().c_str() );
	else if ( elems[0] == "nColumns" )
		noDofs_ = atoi ( elems.back().c_str() )	- 1;
//  	std::cout << "noDofs_: " << noDofs_ << std::endl;

	getline ( dataFile_, s, '\n' );
	dataFile_ >> s;
	std::stringstream ss3 ( s );
	elems.clear();

	while ( std::getline ( ss3, item, '=' ) )
	{
		elems.push_back ( item );
	}


 	std::cout << elems[0] << std::endl;

	if ( elems[0] == "inDegrees" )
	{
		if ( elems.back() == "yes" )
		{
			std::cout << "*************************************************" << std::endl;
			degrees_ = true;
		}
		else if ( elems.back() == "no" )
			degrees_ = false;
		else
		{
			std::cout << "inDegrees line in " << dataFilename << " incorrect " << elems.back() << std::endl;
			exit ( 1 );
		}
	}
// 	else
// 	{
// 		degrees_ = false;
// 	}

// 	std::cout << " ============= " << std::endl;

	if ( s != "endheader" )
	{
		getline ( dataFile_, s );

		if ( s.empty() )
			getline ( dataFile_, s );
	}

	if ( s != "endheader" )
	{
		getline ( dataFile_, s );

		if ( s.empty() )
			getline ( dataFile_, s );
	}

// reading muscles
	string line;
	getline ( dataFile_, line );

	if ( line.empty() )
		getline ( dataFile_, line );

//  	std::cout << "------2------" << std::endl;
	//dataFile_ >> line;
// 	std::cout << line << std::endl;
	stringstream myStream ( line );
	string nextMuscleName;
// the first is the "Time"
	string timeName;
	myStream >> timeName;
// then we have their names

	while ( !myStream.eof() )
	{
		myStream >> nextMuscleName;
		dofsNames_.push_back ( nextMuscleName );
// 		std::cout << nextMuscleName << std::endl;
	}

// 	while ( !myStream.eof() );

// 	std::cout << dofsNames_.back() << " -- " << dofsNames_.at ( dofsNames_.size() - 1 ) << std::endl;

	if ( dofsNames_.back() == dofsNames_.at ( dofsNames_.size() - 1 )  && noDofs_ != dofsNames_.size() )
		dofsNames_.pop_back();

	if ( noDofs_ != dofsNames_.size() )
	{
		cout << "\nSomething is wrong in " << dataFileName_ << endl << noDofs_ << " muscles should be in the file "
				<< "and we have : " << dofsNames_.size() << endl;

		for ( vector<string>::iterator it = dofsNames_.begin(); it != dofsNames_.end(); ++it )
			cout << *it << " \n";

		exit ( EXIT_FAILURE );
	}

	currentData_.resize ( noDofs_ );
	currentDataTime_ = 0.;
	currentTimeStep_ = 0;

}

void DataFromAngleFile::readNextData()
{

	// read time for the data currently stored in DataFromFile
	string line;
	getline ( dataFile_, line );
	stringstream myStream ( line );
	double value;
	currentData_.clear();
	//  cout << "\ndatafromfile check 1\n";
	myStream >> currentDataTime_;

	//    cout << "\ndatafromfile check 1b\n";
	for ( int i = 0; i < dofsNames_.size(); i++ )
	{
		myStream >> value;

		if ( degrees_ )
			value = value / 180 * M_PI;

//		cout << dofsNames_.at(i) << " : " << value << endl;
		currentData_.push_back ( value );
	}

	//    cout << "\ndatafromfile check 2\n";
//	if (currentData_.size() != dofsNames_.size())
//	{
//		cout << "\nERROR: in" << dataFileName_ << " at time step " << currentTimeStep_ << " you have "
//				<< currentData_.size() << " input.\nYou need " << dofsNames_.size() << endl;
//		exit(EXIT_FAILURE);
//	}
	/*
	 dataFile_ >>  currentDataTime_;
	 // and then store their value
	 for (int i = 0; i < noMuscles_; ++i) {
	 double value;
	 dataFile_ >> value;
	 currentData_.at(i) = value;
	 }
	 */
	++currentTimeStep_;
}

const vector<double>& DataFromAngleFile::getCurrentData() const
{
//  if(currentData_.size() == muscleNames_.size())
	return currentData_;
	/*  else
	 {
	 cout << "\nERROR: in" << dataFileName_ << " at time step " << currentTimeStep_ << " you have " << currentData_.size() << " input.\nYou need " << muscleNames_.size() << endl;
	 exit(EXIT_FAILURE);
	 }*/
}

DataFromAngleFile::~DataFromAngleFile()
{
	dataFile_.close();
}
