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

#ifndef EMGDataFromFileRT_h
#define EMGDataFromFileRT_h

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

class EMGDataFromFileRT {
public:
	EMGDataFromFileRT() { std::cout << "You should not be there\n"; }
	EMGDataFromFileRT(const std::string& inputFile);
	void readNextEmgData();
	inline const std::vector<std::string>& getChannelNames() const {return channelNames_;}
	inline double getCurrentTime() const {return currentDataTime_;}
	bool areStillData() const {return currentTimeStep_ < noTimeSteps_;}
	int getNoTimeSteps() const {return noTimeSteps_;}
	const std::vector<double>& getEMGData() {return currentReadEMG_;}
	~EMGDataFromFileRT();
private:
  std::ifstream EMGDataFile_;
  int noChannel_;
  int noTimeSteps_;
  int currentTimeStep_;
  double currentDataTime_;
  std::vector<std::string> channelNames_;
  std::vector<double> currentReadEMG_;
};

#endif

