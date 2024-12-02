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

#ifndef EMGFromX_h
#define EMGFromX_h

#include <vector>
#include "SyncTools.h"

class EMGFromX 
{
  public:
    virtual ~EMGFromX();
    virtual void operator()() {};
    void pushEmgBack(const std::vector<double>& newEmgToPush);
    void updateEmg(const std::vector<double>& currentEmgData, double currentTime);
	void getMusclesNames(std::vector< std::string >& muscleNamesFromModel);
	void getMusclesNamesOnChannel(std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel);
	void getSyncGui(bool& gui);
	void getSyncVerbose(int& verbose);
	void setMusclesNames ( const std::vector< std::string >& musclesNames );
};

#endif
