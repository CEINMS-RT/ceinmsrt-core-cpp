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

#ifndef ModelEvaluationOnline_h
#define ModelEvaluationOnline_h

#include <vector>
#include <string>
#include "ModelEvaluationBase.h"


template <typename NMSmodelT>
class ModelEvaluationOnline : public ModelEvaluationBase {

public:
//    ModelEvaluationOnline();
    ModelEvaluationOnline(NMSmodelT& subject);
//    void setSubject(NMSmodelT& subject);
    void operator()();
    
private:
    NMSmodelT& subject_;
 //   std::vector< std::string > musclesNames_;
 //   std::vector< std::vector < std::string > > musclesNamesOnDof_;
    std::vector< std::string > dofNames_;
    std::vector< std::string > dofNamesWithExtTorque_;
    unsigned noDof_;
};

#include "ModelEvaluationOnline.cpp"

#endif