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
#include "PiecewiseActivation.h"
#include <math.h>

PiecewiseActivation::PiecewiseActivation():
beta1_(0.0), beta2_(0.0), alpha_(0.0), shapeFactor_(0.0),   
emg_(0.0), noOfEmg_(0), pastEmgT1_(0.0), pastEmgT2_(0.0), neuralActivation_(0.0),   
neuralActivationT1_(0.0), neuralActivationT2_(0.0), activation_(0.0), 
a_(0.0), b_(0.0), Xo_(0.0), m_(0.0), c_(0.0)

{
    
}


void PiecewiseActivation::setFilterParameters (double c1, double c2) {

    beta1_ = c1 + c2;
    beta2_ = c1 * c2;
    alpha_ = 1 + beta1_ + beta2_;
    
}


void PiecewiseActivation::setShapeFactor (double shapeFactor) {

    shapeFactor_ = shapeFactor;
    updateActivationParameters();
}


void PiecewiseActivation::setEmg(double emg) {
    
    emg_ = emg;
}


void PiecewiseActivation::updateActivationParameters() {

    double A = shapeFactor_; 
    double Xo = 0.3085-0.7071*A;            // cos(45°)=0.7071
    double Yo = 0.3085+0.7071*A;
    double tol=0.0001;                      //tolerance used in Newton-Raphson method
    int count=0;
    int iter=1000;
    double Po = 0.05; // initial guess
    double m = (1.0-Yo)/(1.0-Xo);
    double c = Yo-m*Xo;
    double a, b, f, fprime, p, expVal;    
   

 // int temp = 0; //remember to remove it
    while (count<iter) {
        expVal = exp(Yo/Po);
        b = (expVal - 1)/Xo;

        // derivative formulation
        f=m-((Po*b)/(b*Xo+1));
        fprime = (1/Xo)*(-1 + ((1 + Yo/Po)/(expVal)));    
        // function formulation
        p = Po-(f/fprime);
        if (fabs(p - Po)<tol) {
        a = Po;  // Po is the "a" value     
    //      temp = count;
        count = iter;
        }
        ++count;
        Po = p;
    }
    Xo_ = Xo;
    m_ = m;       
    c_ = c;
    a_ = a;
    b_ = b;
}


void PiecewiseActivation::updateActivation() {

    if ( noOfEmg_ == 2 ) {
        neuralActivationT1_ = (pastEmgT2_ + pastEmgT1_)/2;
        neuralActivationT2_ = neuralActivationT1_;
    } 
    
    neuralActivation_ = (alpha_*emg_) - (beta1_*neuralActivationT1_) - (beta2_*neuralActivationT2_);
    // Account for linear or non-linear u(t) -> a(t)
    // The (Xo,Yo) node point is common to both linear and non-linear
    // portion of u(t)-a(t) relationship
 
    // non-linear region
    if (neuralActivation_ < Xo_) {
        if (((b_*neuralActivation_)+1) > 0) 
            activation_ = a_*log((b_*neuralActivation_)+1);
        else activation_ = 0;     
    }                     
    else // linear region
        activation_ = m_* neuralActivation_+c_;   
 
}



void PiecewiseActivation::pushState() {

    neuralActivationT2_ = neuralActivationT1_;
    neuralActivationT1_ = neuralActivation_;
    pastEmgT2_ = pastEmgT1_;
    pastEmgT1_ = emg_;
    noOfEmg_++;
}


void PiecewiseActivation::resetState() {

    emg_ = .0;
    activation_ = .0;
    neuralActivation_ = .0;
    neuralActivationT1_ = .0;
    neuralActivationT2_ = .0;
    pastEmgT1_ = .0;
    pastEmgT2_ = .0;
    noOfEmg_ = 0;
}

PiecewiseActivation::~PiecewiseActivation()
{

}





