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

#include "EMGDataFromFile.h"

template <typename EMGgenerator>
EMGDataFromFile<EMGgenerator>::EMGDataFromFile(const string& EMGDataFilename)
:EMGDataFile_(EMGDataFilename.c_str()) {
 
  if (!EMGDataFile_.is_open()) {
    cout << "ERROR: " << EMGDataFilename << " could not be open\n";
    exit(EXIT_FAILURE);
  }
  
  // reading number of columns/rows
  string trash;
  EMGDataFile_ >> trash; 
  int noColumns;
  EMGDataFile_ >> noColumns;
  noMuscles_ = noColumns-1;
  EMGDataFile_ >> trash;
  EMGDataFile_ >> noTimeSteps_;

  // reading muscles
  string line;
  getline(EMGDataFile_, line, '\n'); getline(EMGDataFile_, line, '\n');  
  stringstream myStream(line);
  string nextMuscleName;
  // --- Read Interpolation Data
  string timeName;
  myStream >> timeName;
  // 1. first their names
  do {
    myStream >> nextMuscleName;
    muscleNames_.push_back(nextMuscleName); 
  } while (!myStream.eof());

  if (noMuscles_ != muscleNames_.size()) {
    cout << "Something is wrong. " << noMuscles_ << " muscles should be in the file "
         << "and we have : " << muscleNames_.size();
    exit(EXIT_FAILURE);
  } 
  
  if ( !EMGgenerator_.checkFromMusclesNames(muscleNames_) ) {
    cout << "THE EMG generator is not able to generate EMG for your muscles starting from muscle in the file! (muscle names are different)\n";
    exit(EXIT_FAILURE);
  }
  
//TODO fix resize
  currentReadEMG_.resize(EMGgenerator_.getNoFromMuscles());
  currentEMGData_.resize(EMGgenerator_.getNoToMuscles());
  currentDataTime_ = 0.;
  currentTimeStep_ = 0;
}

template <typename EMGgenerator>
void EMGDataFromFile<EMGgenerator>::readNextEmgData()  {

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
  EMGgenerator_.convert(currentReadEMG_, currentEMGData_);
 
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

template <typename EMGgenerator>
EMGDataFromFile<EMGgenerator>::~EMGDataFromFile() {
  EMGDataFile_.close();
}


#include "policyTemplates.h"

