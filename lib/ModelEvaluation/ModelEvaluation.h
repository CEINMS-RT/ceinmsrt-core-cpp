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

#ifndef ModelEvaluation_h
#define ModelEvaluation_h

#include <vector>
#include <string>
#include "NMSmodel.h"
#include "SetupDataStructure.h"

#include "ExponentialActivation.h"
#include "StiffTendon.h"
#include "ElasticTendon.h"
#include "ElasticTendon_BiSec.h"

typedef StiffTendon MyTendonType;


class ModelEvaluation
{
  public:
    typedef NMSmodel<ExponentialActivation, MyTendonType> MyNMSmodel;
    typedef SetupDataStructure<ExponentialActivation, MyTendonType> MySetupDataStructure;
    ModelEvaluation(const std::string& configurationFile);
    ~ModelEvaluation();
    void operator()();
  private:
    void popEmgFront(std::vector<double>& emgFromQueue);
    void popLmtFront(std::vector<double>& lmtFromQueue);
    void popMomentArmsFront(std::vector<double>& momentArmsFromQueue, unsigned int whichDof);
    void popExternalTorqueFront(std::vector<double>& externalTorqueFromQueue, unsigned int whichDof);    
    void getMusclesNames();
    void setDofNames();
    void getDofNamesAssociatedToExternalTorque();
    void initOfflineCurve(MyNMSmodel& mySubject);
    std::string configurationFile_;
    std::vector< std::string > musclesNames_;
    std::vector< std::vector < std::string > > musclesNamesOnDof_;
    std::vector< std::string > dofNames_;
    std::vector< std::string > dofNamesWithExtTorque_;
    unsigned int noDof_;
};


#endif
