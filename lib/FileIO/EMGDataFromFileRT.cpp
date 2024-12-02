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

#include "EMGDataFromFileRT.h"

EMGDataFromFileRT::EMGDataFromFileRT(const string& EMGDataFilename)
:EMGDataFile_(EMGDataFilename.c_str()) {
 
  if (!EMGDataFile_.is_open()) {
    cout << "ERROR: " << EMGDataFilename << " could not be open\n";
    exit(EXIT_FAILURE);
  }
  
  // reading number of columns/rows
//  string trash;
//  EMGDataFile_ >> trash;
//  int noColumns;
//  EMGDataFile_ >> noColumns;
//  noChannel_ = noColumns-1;
//  EMGDataFile_ >> trash;
//  EMGDataFile_ >> noTimeSteps_;
//
//  // reading muscles
//  string line;
//  getline(EMGDataFile_, line, '\n'); getline(EMGDataFile_, line, '\n');
//  stringstream myStream(line);
//  string nextMuscleName;
//  // --- Read Interpolation Data
//  string timeName;
//  myStream >> timeName;
	string line;
	getline(EMGDataFile_, line, '\n');
	getline(EMGDataFile_, line, '\n');
	{
		std::vector<string> elems;
		std::stringstream ss(line);
		string item;
		while (std::getline(ss, item, '='))
		{
			if(item != " " || item != "\t")
				elems.push_back(item);
		}
		noChannel_ = atof(elems.back().c_str()) - 1;
	}
	getline(EMGDataFile_, line, '\n');
	{
		std::vector<string> elems;
		std::stringstream ss(line);
		string item;
		while (std::getline(ss, item, '='))
		{
			if(item != " " || item != "\t")
				elems.push_back(item);
		}
		noTimeSteps_ = atof(elems.back().c_str()) - 1;
	}
	getline(EMGDataFile_, line, '\n');
	getline(EMGDataFile_, line, '\n');
//	std::cout << line << std::endl;
	stringstream myStream(line);
	string nextMuscleName;
	// --- Read Interpolation Data
	string timeName;
	myStream >> timeName;
  // 1. first their names
  do {
    myStream >> nextMuscleName;
//    std::cout << nextMuscleName << std::endl;
    channelNames_.push_back(nextMuscleName);
  } while (!myStream.eof());
  if(channelNames_.back() == channelNames_.at(channelNames_.size() - 1))
	  channelNames_.pop_back();

//  noChannel_ = channelNames_.size();

  if (noChannel_ != channelNames_.size()) {
    cout << "Something is wrong. " << noChannel_ << " muscles should be in the file "
         << "and we have : " << channelNames_.size();
    exit(EXIT_FAILURE);
  }
  
//TODO fix resize
  currentReadEMG_.resize(noChannel_);
  currentDataTime_ = 0.;
  currentTimeStep_ = 0;
}

void EMGDataFromFileRT::readNextEmgData()  {

  // first we build a vector of emg read from the file
 
  string line;
  getline(EMGDataFile_, line, '\n');
  stringstream myStream(line);
  double value;
  currentReadEMG_.clear();
  myStream >>  currentDataTime_;
 // cout << "EMGdatafromfile in emg.txt: time step "<< currentDataTime_ << endl;
  do {
    myStream >> value;
  //  cout << "EMGdatafromfile in emg.txt: value "<< value << endl;
    currentReadEMG_.push_back(value); 
  } while (!myStream.eof());
 // cout << "EMGDataFromFile: just read " << currentReadEMG_.size() << " values\n"; 
  currentReadEMG_.pop_back();
/*
  int noFromMuscles = EMGgenerator_.getNoFromMuscles();
  // 1. first their names
  for (int i = 0; i < noFromMuscles; ++i) {
    double value;
    EMGDataFile_ >> value; 
    currentReadEMG_[i] = value;
  } 

  // then we convert them on what they need
  EMGgenerator_.convert(currentReadEMG_, currentEMGData_);
*/
  ++currentTimeStep_;

}

EMGDataFromFileRT::~EMGDataFromFileRT() {
  EMGDataFile_.close();
}

