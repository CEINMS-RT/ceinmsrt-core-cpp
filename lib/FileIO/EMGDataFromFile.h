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

#ifndef EMGDataFromFile_h
#define EMGDataFromFile_h

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

template <typename EMGgenerator>
class EMGDataFromFile {
public:
  EMGDataFromFile() { std::cout << "You should not be there\n"; }
  EMGDataFromFile(const std::string& inputFile);
  const std::vector<std::string>& getMusclesNames() const { return EMGgenerator_.getMusclesNames(); }
  void getMusclesNames( std::vector< std::string >& muscleNames ) const {muscleNames = EMGgenerator_.getMusclesNames(); }
  void readNextEmgData();
  inline double getCurrentTime() const {return currentDataTime_;}
  bool areStillData() const { return currentTimeStep_ < noTimeSteps_; } 
  const std::vector<double>& getCurrentData() const {return currentEMGData_;}
  int getNoTimeSteps() const {return noTimeSteps_;}
  ~EMGDataFromFile();
private:
  std::ifstream EMGDataFile_;
  int noMuscles_;
  int noTimeSteps_;
  int currentTimeStep_;
  double currentDataTime_;
  std::vector<std::string> muscleNames_;
  std::vector<double> currentReadEMG_;
  std::vector<double> currentEMGData_;
  EMGgenerator EMGgenerator_;
};

//#include "EMGDataFromFile.cpp"

#endif

