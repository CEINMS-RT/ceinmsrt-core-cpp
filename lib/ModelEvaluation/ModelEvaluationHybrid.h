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

#ifndef ModelEvaluationHybrid_h
#define ModelEvaluationHybrid_h

#include <vector>
#include <string>
#include "ModelEvaluationBase.h"

template <typename NMSmodelT, typename ErrorMinimizerT>
class ModelEvaluationHybrid : public ModelEvaluationBase {

public:
    ModelEvaluationHybrid(NMSmodelT& subject, ErrorMinimizerT& torqueErrorMinimizer);
    void operator()();

private:
    NMSmodelT& subject_;
    ErrorMinimizerT& torqueErrorMinimizer_;
    std::vector< std::string > dofNames_;
    std::vector< std::string > dofNamesWithExtTorque_;
    unsigned noDof_;
};

#include "ModelEvaluationHybrid.cpp"

#endif
