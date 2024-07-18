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

#ifndef Curve_h
#define Curve_h

#include <vector>
#include "CircularVector.h"
#include <boost/timer/timer.hpp>

namespace CurveMode {
    enum Mode{Online, Offline};
};


template <CurveMode::Mode mode, typename T, typename U>
struct Select {

   typedef T Result;
};


template <typename T, typename U>
struct Select<CurveMode::Online, T, U> {

   typedef U Result;
}; 

template <typename T, typename U>
struct Select<CurveMode::Offline, T, U> {

	typedef U Result;
};

template <CurveMode::Mode mode>
class Curve;


template <CurveMode::Mode mode>
std::ostream& operator<< (std::ostream& output, const Curve<mode>& c);


template <CurveMode::Mode mode>
class Curve {

public:
    Curve();
    // compute coefficients
    Curve(const std::vector<double>& x, const std::vector<double> &y);
    Curve(const Curve& orig);
    virtual ~Curve() {}
    Curve& operator=(const Curve& orig);
    void reset();
    // add a new points and compute again coefficients
    void addPoint(double x, double y);
    void addPointOnly(double x, double y);
    void resetPointsWith(const std::vector<double>& x, const std::vector<double> &y);
    void refresh(); 
    // remove last point of the Curve_c and compute again coefficients
    void removeLastPoint();
    void removeLastPointNoUpdate(); //remove last point without computing the coefficients again
    // interpolation 
    double getValue(double xValue) const;
    double getFirstDerivative(double xValue) const;
    double getSecondDerivative(double xValue) const;
    friend std::ostream& operator<< <>(std::ostream& output, const Curve& c);
    unsigned getSize();
private:
    void computeCoefficients();

    typedef typename Select<mode, std::vector<double>, CircularVector<double> >::Result VectorType;  
    VectorType x_;
    VectorType y_;
    std::vector<double>    b_;
    std::vector<double>    c_;
    std::vector<double>    d_;

};

#include "Curve.cpp"

#endif
