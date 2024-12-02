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

#ifndef ModelEvaluationBase_h
#define ModelEvaluationBase_h

#include <vector>
#include <string>
#include <map>

class ModelEvaluationBase {
    
public:  
    virtual ~ModelEvaluationBase();
    virtual void operator()() = 0;
    
    void getEmgFromShared(std::vector<double>& emgFromQueue);
    void getLmtFromShared(std::vector<double>& lmtFromQueue);
    void getAngleFromShared(std::vector<double>& angleFromQueue);
    void getMomentArmsFromShared(std::vector<double>& momentArmsFromQueue, unsigned int whichDof);
    void getExternalTorqueFromShared(std::vector<double>& externalTorqueFromQueue);    
    void getMusclesNamesFromShared(std::vector<std::string>& muscleNames);
    void getMusclesNamesOnDofsFromShared(std::vector< std::vector<std::string> >& muscleNamesOnDofs);
    void setDofNamesToShared(const std::vector<std::string>& dofNames);
    void setMuscleNamesToShared(const std::vector<std::string>& muscleNames);
    void getDofNamesAssociatedToExternalTorque(std::vector<std::string>& dofNames);
    void setMusclesNamesOnChannel(std::map<std::string, std::vector <std::string> > musclesNamesOnChannel);
	void getSyncGui(bool& gui);
	void getSyncVerbose(int& verbose);
};

#endif
