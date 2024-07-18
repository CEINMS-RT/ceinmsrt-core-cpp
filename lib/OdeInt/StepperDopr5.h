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
#ifndef StepperDopr5_h
#define StepperDopr5_h

#include <CommonCEINMS.h>
#include <vector>
#include <limits>
#include <cmath>
#include "StepperBase.h"
#include <iostream>
#include <stdlib.h>
using std::max;
using std::min;
#include <algorithm>

// template<typename T>
// inline const T &max(const T &a, const T &b)
//         {return b > a ? (b) : (a);}
// 
// template<typename T>
// inline const T &min(const T &a, const T &b)
//         {return b > a ? (a) : (b);}
        
template <typename D>
class StepperDopr5 : public StepperBase {

public:
    StepperDopr5(std::vector<double>& y, std::vector<double>& dydx, double& x,
                 const double aTol, const double rTol, bool dense);
    
    typedef D Dtype;
    
    void step(const double hTry, D& derivs);
    void dy(const double h, D& derivs);
    void prepare_dense(const double h, D& derivs);
    double dense_out(const int i, const double x, const double h);
    double error();
    
    
private:
    std::vector<double> k2_, k3_, k4_, k5_, k6_;
    std::vector<double> rCont1_, rCont2_, rCont3_, rCont4_, rCont5_;
    std::vector<double> dydxNew_;
 
    class Controller {
    public:
        double hNext_, errOld_;
        bool reject_;
        Controller();
        bool success(const double err, double& h);
    };
    Controller con_;
};


template <class D>
StepperDopr5<D>::StepperDopr5(std::vector<double>& y, std::vector<double>& dydx, double& x,
                 const double aTol, const double rTol, bool dense) :
StepperBase(y, dydx, x, aTol, rTol, dense), 
k2_(y.size()),
k3_(y.size()),
k4_(y.size()),
k5_(y.size()),
k6_(y.size()),
rCont1_(y.size()),
rCont2_(y.size()),
rCont3_(y.size()),
rCont4_(y.size()),
rCont5_(y.size()),
dydxNew_(y.size()) {

    EPS_ = std::numeric_limits<double>::epsilon();
	//COUT << y_.size() << std::endl;
	//for (unsigned i = 0; i < n_ ; i++)
	//	COUT << y_[i] << std::endl;
}

template <class D>
void StepperDopr5<D>::step(const double hTry, D& derivs) {

    double h = hTry;
    double err = 0;
	int cpt = 0;
	
	//COUT << " hInit: " << h << std::endl;
    do {
        dy(h,derivs);
        err = error();
		//COUT << "err: " << err << " h: " << h << std::endl;
        if (fabs(h) <= fabs(x_)*EPS_) {
            COUT << "stepsize underflow in StepperDopr5\n";
            exit(EXIT_FAILURE);
        }
        cpt++;
		if(cpt > 10000)
		{
 			COUT << "Too much step StepperDopr5" << std::endl;
			COUT << "err: " << err << " h: " << h << std::endl;
			break;
//             exit(EXIT_FAILURE);
		}
    } while(!con_.success(err, h) );
    if (dense_)
        prepare_dense(h, derivs);
    dydx_ = dydxNew_;
    y_ = yOut_;
    xOld_ = x_;
    x_ += (hDid_ = h);
    hNext_ = con_.hNext_;
}



template <class D>
void StepperDopr5<D>::dy(const double h, D& derivs) {

    
    const double c2=0.2,c3=0.3,c4=0.8,c5=8.0/9.0,a21=0.2,a31=3.0/40.0,
    a32=9.0/40.0,a41=44.0/45.0,a42=-56.0/15.0,a43=32.0/9.0,a51=19372.0/6561.0,
    a52=-25360.0/2187.0,a53=64448.0/6561.0,a54=-212.0/729.0,a61=9017.0/3168.0,
    a62=-355.0/33.0,a63=46732.0/5247.0,a64=49.0/176.0,a65=-5103.0/18656.0,
    a71=35.0/384.0,a73=500.0/1113.0,a74=125.0/192.0,a75=-2187.0/6784.0,
    a76=11.0/84.0,e1=71.0/57600.0,e3=-71.0/16695.0,e4=71.0/1920.0,
    e5=-17253.0/339200.0,e6=22.0/525.0,e7=-1.0/40.0;

    std::vector<double> yTemp(n_);

    for(unsigned i=0; i < n_; ++i) //1st
        yTemp.at(i) = y_.at(i) + h*a21*dydx_.at(i);

    derivs(x_ + c2*h, yTemp, k2_);  //2nd
    for(unsigned i = 0; i < n_; ++i)
        yTemp.at(i) = y_.at(i) + h*(a31*dydx_.at(i) + a32*k2_.at(i));

    derivs(x_ + c3*h, yTemp, k3_); //3rd
    for(unsigned i = 0; i < n_; ++i)
        yTemp.at(i) = y_.at(i) + h*(a41*dydx_.at(i) + a42*k2_.at(i) + a43*k3_.at(i));

    derivs(x_ + c4*h, yTemp, k4_); //4th
    for(unsigned i = 0; i < n_; ++i)
        yTemp.at(i) = y_.at(i) + h*(a51*dydx_.at(i) + a52*k2_.at(i) + a53*k3_.at(i) + a54*k4_.at(i));
    
    derivs(x_ + c5*h, yTemp, k5_); //5th
    for(unsigned i = 0; i < n_; ++i)
        yTemp.at(i) = y_.at(i) + h*(a61*dydx_.at(i) + a62*k2_.at(i) + a63*k3_.at(i) + a64*k4_.at(i) + a65*k5_.at(i));

    double xph = x_ + h;           //6th
    derivs(xph, yTemp, k6_); 
    for(unsigned i = 0; i < n_; ++i)
        yOut_.at(i) = y_.at(i) + h*(a71*dydx_.at(i) + a73*k3_.at(i) + a74*k4_.at(i) + a75*k5_.at(i) + a76*k6_.at(i));

    derivs(xph, yOut_, dydxNew_);

    for(unsigned i = 0; i < n_; ++i)
        yErr_.at(i) = h*(e1*dydx_.at(i) + e3*k3_.at(i) + e4*k4_.at(i) + e5*k5_.at(i) + e6*k6_.at(i) + e7*dydxNew_.at(i));

}

    
template <class D>
void StepperDopr5<D>::prepare_dense(const double h, D& derivs) {

 
    std::vector<double> yTemp(n_);
    const double d1=-12715105075.0/11282082432.0,
                 d3=87487479700.0/32700410799.0, d4=-10690763975.0/1880347072.0,
                 d5=701980252875.0/199316789632.0, d6=-1453857185.0/822651844.0,
                 d7=69997945.0/29380423.0;

    for (unsigned i = 0; i < n_; ++i) {
        rCont1_.at(i) = y_.at(i);
        double  yDiff = yOut_.at(i) - y_.at(i);
        rCont2_.at(i) = yDiff;
        double bspl = h*dydx_.at(i) - yDiff;
        rCont3_.at(i) = bspl;
        rCont4_.at(i) = yDiff - h*dydxNew_.at(i) - bspl;
        rCont5_.at(i) = h*(d1*dydx_.at(i) + d3*k3_.at(i) + d4*k4_.at(i) + d5*k5_.at(i) + d6*k6_.at(i) + d7*dydxNew_.at(i));
    }    
}


template <class D>
double StepperDopr5<D>::dense_out(const int i, const double x, const double h) {
    
    double s = (x - xOld_)/h;
    double s1 = 1.0 - s;
    return rCont1_.at(i) + s*(rCont2_.at(i) + s1*(rCont3_.at(i) + s*(rCont4_.at(i) + s1*rCont5_.at(i))));
}


template <class D>
double StepperDopr5<D>::error() {
 
    double err = 0.0, sk;
	//COUT << "n_; " << n_ << " aTol_: " << aTol_ << " rTol_: " << rTol_ << std::endl;
    for (unsigned i = 0; i < n_ ; i++) {
        sk = aTol_ + rTol_*max<double>(fabs(y_.at(i)), fabs(yOut_.at(i)));
		//COUT << "y_.at(i): " << y_.at(i) << " yOut_.at(i): " << yOut_.at(i) << std::endl;
    err += ( (yErr_.at(i)/sk) *  (yErr_.at(i)/sk) );
    }
    return sqrt(err/n_);
}


template <class D>
StepperDopr5<D>::Controller::Controller() : reject_(false), errOld_(1.0e-4) {}


template <class D>
bool StepperDopr5<D>::Controller::success(const double err, double& h) {
 
    const double beta = 0.0; 
    const double alpha = 0.2 - beta*0.75;
    const double safe = 0.9;
    const double minscale = 0.2;
    const double maxscale = 10.0;

    double scale;
    if (err <= 1.0) {
        if (err == 0.0)
            scale = maxscale;
        else {
            scale = safe*pow(err, -alpha)*pow(errOld_,beta);
            if (scale < minscale) scale = minscale;
            if (scale > maxscale) scale = maxscale;
        }
        if (reject_)
            hNext_ = h*min(scale,1.0);
        else
            hNext_ = h*scale;
        errOld_ = max(err,1.0e-4);

        reject_ = false;
        return true;
    } else {
        scale = max(safe*pow(err,-alpha),minscale);
        h *= scale;
        reject_ = true;
        return false;
    }       
}
    
    
    
    
    
    
    
    
    
    
    
#endif