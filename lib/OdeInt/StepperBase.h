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

#ifndef Stepperbase_h
#define StepperBase_h

#include <vector>

class StepperBase {
public:
    StepperBase(std::vector<double>& y, std::vector<double>& dydx, double& x, const double aTol,
                const double rTol, bool dense) : x_(x), y_(y), dydx_(dydx), aTol_(aTol),
    rTol_(rTol), dense_(dense), n_((int)y.size()) ,nEqn_((int)y.size()), yOut_(y.size()),yErr_(y.size()) {}
    
    double& x_;
    double xOld_;
    std::vector<double>& y_;
    std::vector<double>& dydx_;
    double aTol_, rTol_;
    bool dense_;
    double hDid_;
    double hNext_;
    double EPS_;
    int n_, nEqn_;
    std::vector<double> yOut_, yErr_;
};

#endif