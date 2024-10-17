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

