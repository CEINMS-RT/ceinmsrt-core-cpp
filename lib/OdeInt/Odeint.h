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
#ifndef Odeint_h
#define Odeint_h

#include <vector>
#include <limits>
#include <math.h>
#include <iostream>
#include "Output.h"

inline float sign(const double &a, const double &b)
        {return b >= 0 ? (a >= 0 ? (float)a : (float)-a) : (a >= 0 ? (float)-a : (float)a);}

        
template<typename Stepper>
class Odeint {

public:
    Odeint(std::vector<double>& yStart, const double x1, const double x2,
           const double aTol, const double rTol, const double h1,
           const double hMin, Output& out, typename Stepper::Dtype &derivs);
    void integrate();

private:
    static const unsigned MAXSTP_ = 50000;
    double EPS_;
    int nok_;
    int nbad_;
    int nvar_;
    double x1_, x2_, hMin_;
    bool dense_;
    std::vector<double> y_, dydx_;
    std::vector<double> &yStart_;
    Output &out_;
    typename Stepper::Dtype &derivs_;
    Stepper s_;
    unsigned nstp_;
    double x_, h_;
};




template<typename Stepper>
Odeint<Stepper>::Odeint(std::vector<double>& yStart, const double x1, const double x2,
                        const double aTol, const double rTol, const double h1,
                        const double hMin, Output& out, typename Stepper::Dtype &derivs): 
nvar_(yStart.size()),
y_(yStart.size()),
dydx_(yStart.size()),
yStart_(yStart),
x_(x1),
nok_(0),
nbad_(0),
x1_(x1),
x2_(x2),
hMin_(hMin),
dense_(out.dense_),
out_(out),
derivs_(derivs),
s_(y_, dydx_, x_, aTol, rTol, out.dense_) {
        
    EPS_ = std::numeric_limits<double>::epsilon();
    h_ = sign(h1, x2-x1);
    for (unsigned i = 0; i < nvar_; ++i) y_.at(i) = yStart.at(i);
    out_.init(s_.nEqn_, x1_, x2_);
}



template<class Stepper>
void Odeint<Stepper>::integrate() {
    
    derivs_(x_, y_, dydx_);
    if (dense_)
        out_.out(-1, x_, y_, s_, h_);
    else
        out_.save(x_, y_);
    for(unsigned nstp = 0; nstp < MAXSTP_; ++nstp) {
        
        if((x_+h_*1.0001 - x2_)*(x2_ - x1_) > 0.0)
            h_ = x2_ - x_;
            s_.step(h_, derivs_);
        if(s_.hDid_ == h_) ++nok_; else ++nbad_;
        if(dense_)
            out_.out(nstp_, x_, y_, s_, s_.hDid_);
        else
            out_.save(x_, y_);
        if ((x_ - x2_)*(x2_ - x1_) >= 0.0) {
            for(unsigned i = 0; i < nvar_; i++) yStart_.at(i) = y_.at(i);
            if (out_.kMax_ > 0 && fabs(out_.xSave_.at(out_.count_-1) - x2_) > 100.0*fabs(x2_)*EPS_)
                out_.save(x_,y_);
            return;
        }
        if (fabs(s_.hNext_) <= hMin_) throw("Step size too small in Odeint");
        h_ = s_.hNext_;
    }
    throw("Too many steps in routine Odeint");
}

#endif