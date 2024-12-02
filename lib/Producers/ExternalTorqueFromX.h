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

#ifndef ExternalTorqueFromX_h
#define ExternalTorqueFromX_h

#include <vector>
#include <string>

class ExternalTorqueFromX
{
  public:
    virtual ~ExternalTorqueFromX();
    virtual void operator()() {};
    void pushExternalTorqueBack(const std::vector<double>& newExternalTorqueToPush);
    void updateExternalTorque(const std::vector<double>& currentExternalTorqueData, double currentTime); 
    void getDofNames(std::vector< std::string >& dofNamesFromModel);
    void setExternalTorqueDofNames(std::vector<std::string> dofNamesWithExternalTorqueFromInput);
   
};

#endif
