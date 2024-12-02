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
