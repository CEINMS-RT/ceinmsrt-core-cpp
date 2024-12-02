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

#include "Curve.h"

#include <vector>
using std::vector;

//const static int pointsNumber = 17;
template <CurveMode::Mode mode>
Curve<mode>::Curve() {
  // make a local copy of the points
  x_.clear();
  y_.clear();
  b_.resize(0);
  c_.resize(0);
  d_.resize(0);
    
}

/* CALC_SPLINE_COEEFICIENTS: this routine takes an array of x and y values,
* and computes the coefficients of natural splines which interpolate the data.
* The code is translated from a Fortran version printed in "Computer
* Methods for Mathematical Computations" by Forsythe, Malcolm, and
* Moler, pp 77-8.
*/

template <CurveMode::Mode mode>
Curve<mode>::Curve(const vector<double>& x, const vector<double> &y) {

  x_ = x;
  y_ = y;
  
  computeCoefficients();
}


template <CurveMode::Mode mode>
Curve<mode>::Curve(const Curve<mode>& orig) {
  x_ = orig.x_;
  y_ = orig.y_;
  b_ = orig.b_;
  c_ = orig.c_;
  d_ = orig.d_;
};


template <CurveMode::Mode mode>
Curve<mode>& Curve<mode>::operator=(const Curve<mode>& orig) {
  x_ = orig.x_;
  y_ = orig.y_;
  b_ = orig.b_;
  c_ = orig.c_;
  d_ = orig.d_;
  return *this;
}


template <CurveMode::Mode mode>
void Curve<mode>::reset() {

  x_.clear();
  y_.clear();
  b_.resize(0);
  c_.resize(0);
  d_.resize(0);
}


template <CurveMode::Mode mode>
void Curve<mode>::resetPointsWith(const vector<double>& x, const vector<double> &y) {
  // make a local copy of the points
  x_ = x;
  y_ = y;
  
  computeCoefficients();
}


template <CurveMode::Mode mode>
void Curve<mode>::removeLastPoint()
{
  x_.pop_back();
  y_.pop_back();

  computeCoefficients();

}


template <CurveMode::Mode mode>
void Curve<mode>::removeLastPointNoUpdate() {
//  std::cout <<"remove" << std::endl << std::flush;
	//std::cout << x_.size() << std::endl << std::flush;
	x_.pop_back();
    y_.pop_back();
	//std::cout << x_.size() << std::endl << std::flush;
}


template <CurveMode::Mode mode>
void Curve<mode>::addPoint(double x, double y)  {
/*	std::cout <<"ok" << std::endl << std::flush;
	if(x_.size() < 1000)
	{*/
//	std::cout << "ok1 " << x_.size() << std::endl << std::flush;
  x_.push_back(x);
  y_.push_back(y);
// 	}
  computeCoefficients();
}


template <CurveMode::Mode mode>
void Curve<mode>::addPointOnly(double x, double y)
{
	
// 	std::cout <<"ok" << std::endl << std::flush;
//   if(x_.size() < 1000)
// 	{
//	std::cout << "ok2 " << x_.size() << std::endl << std::flush;
  x_.push_back(x);
  y_.push_back(y);
// 	}
}


template <CurveMode::Mode mode>
void Curve<mode>::refresh()
{
  computeCoefficients();
}

  
template <CurveMode::Mode mode>
void Curve<mode>::computeCoefficients() {
  
  unsigned n = x_.size();
  int nm1 = n-1;
 
   
  b_.resize(n); 
  c_.resize(n);  
  d_.resize(n);  

 
  if (n > 2) {
    // setup tridiagonal system:
    // b_ = diagonal
    // d_ = offdiagonal
    // c_ = right-hand side
	  d_.at(0) = x_.at(1) - x_.at(0);
	  c_.at(1) = ( y_.at(1) - y_.at(0) ) / d_.at(0);
	 
	  for (unsigned i = 1; i < (unsigned) nm1; ++i ) {
	    d_.at(i) = x_.at(i+1) - x_.at(i);
		  b_.at(i) = 2.0*(d_.at(i-1) + d_.at(i));
		  c_.at(i+1) = (y_.at(i+1) - y_.at(i))/ d_.at(i);
		  c_.at(i) = c_.at(i+1) - c_.at(i);
	  }  
	  
	  // End conditions. Third derivatives at x_.at(0) and x_.at(n-1)
	  // are obtained from divided differences. 
	
	  b_.at(0)   = -d_.at(0);
	  b_.at(nm1) = -d_.at(nm1-1);  
	  c_.at(0)   = 0.;
	  c_.at(nm1) = 0.; 
	  
	  if (n > 3) {
	    c_.at(0) = c_.at(2) / ( x_.at(3)-x_.at(1) ) - c_.at(1) / ( x_.at(2)- x_.at(0) );
		  c_.at(nm1) = c_.at(nm1-1)/(x_.at(nm1)-x_.at(n-3)) -
			             c_.at(n-3)/( x_.at(nm1-1) - x_.at(n-4));
		  c_.at(0) = c_.at(0) * d_.at(0) * d_.at(0) / ( x_.at(3) - x_.at(0) );
		  c_.at(nm1) = -c_.at(nm1) * d_.at(nm1-1) * d_.at(nm1-1)/(x_.at(nm1) - x_.at(n-4));
	  } 

	  // Forward elimination 
	  for (unsigned i=1; i < n; ++i) {
		  double t = d_.at(i-1)/b_.at(i-1);
		  b_.at(i) -= t * d_.at(i-1);
		  c_.at(i) -= t * c_.at(i-1);
	  }
	
	  // Back substitution
	  c_.at(nm1) /= b_.at(nm1);
	  for (int j=0; j<nm1; ++j) {
		  int i = nm1 - 1 - j;
		  c_.at(i) = (c_.at(i) - d_.at(i) * c_.at(i+1))/b_.at(i);
	  }
	
	  // compute polynomial coefficients
	  b_.at(nm1) = (y_.at(nm1) - y_.at(nm1-1))/ d_.at(nm1-1) +
		             d_.at(nm1-1) * (c_.at(nm1-1) + 2.0* c_.at(nm1));
		             
	  for (int i = 0; i < nm1; ++i) {
		  b_.at(i) = (y_.at(i+1) - y_.at(i)) / d_.at(i) - d_.at(i) * ( c_.at(i+1) + 2.0 * c_.at(i) );
		  d_.at(i) = (c_.at(i+1) - c_.at(i)) / d_.at(i); 
		  c_.at(i) *= 3.0; 
	  }

	  c_.at(nm1) *= 3.0;
	  d_.at(nm1) = d_.at(nm1-1);
	  
  }
  
  if (n == 2) {
	 // std::cout << "--:: " << y_.at(1) << " " << y_.at(0) << " " << std::flush;
	//  std::cout << x_.at(1) << " " << x_.at(0) << std::flush;
    b_.at(0) = ( y_.at(1)-y_.at(0) ) / ( x_.at(1)-x_.at(0) );
    b_.at(1) = b_.at(0); 
    c_.at(0) = 0.;
    c_.at(1) = 0.;
    d_.at(0) = 0.;
    d_.at(1) = 0.; 
  } 

 // std::cout << "++:: " << n << " " << std::flush;
 // std::cout << d_.at(0) << " " << std::flush;
 // std::cout << c_.at(0) << " " << std::flush;
 // std::cout << b_.at(0) << " " <<  std::endl << std::flush;
  
   // do nothing if size < 2
   
}


/*******************************************************************************/
/* INTERPOLATE_SPLINE: given a spline function and an x-value, this routine
* finds the corresponding y-value by interpolating the spline. It
* can return the zeroth, first, or second derivative of the spline
* at that x-value.
*/
template <CurveMode::Mode mode>
double Curve<mode>::getValue(double xalue) const {

  int n = x_.size();
  
  
	if ( xalue < x_.at(0)) 
	  return y_.at(0);
	if ( xalue > x_.at(n-1))  
	  return y_.at(n-1); 
	  
	
  if (n == 1) 
    return x_.at(0);
    
  int k = 0; 
  if (n == 2)
    k = 0;    
  else {  
	  // Do a binary search to find which two spline control points the abscissa
	  // is between   
	
	  int i = 0;
	  int j = n;
	 
	  while (1)
	  {
		  k = (i+j)/2;
		  if (xalue < x_.at(k))
			  j = k;
		  else if (xalue > x_.at(k+1))
			  i = k;
		  else
			  break;
	   }
	}
	
	double dx = xalue - x_.at(k);
	/*std::cout << "++:: " << dx << " " << k << " " << x_.size() << " " << std::flush;
	std::cout << d_.at(k) << " " << std::flush;
	std::cout << c_.at(k) << " " << std::flush;
	std::cout << b_.at(k) << " " << std::flush;
	std::cout << (y_.at(k) + dx * (b_.at(k) + dx * (c_.at(k) + dx * d_.at(k)))) << " " << std::endl << std::flush;*/
	
	return (y_.at(k) + dx * ( b_.at(k) + dx * ( c_.at(k) + dx * d_.at(k) ) ) );
}

/*******************************************************************************/
/* INTERPOLATE_SPLINE: given a spline function and an x-value, this routine

* finds the corresponding y-value by interpolating the spline. It
* can return the zeroth, first, or second derivative of the spline
* at that x-value.
*/

template <CurveMode::Mode mode>
double Curve<mode>::getFirstDerivative(double xalue) const  {

//  	boost::timer::auto_cpu_timer auto_t;
	int n = x_.size();
	
// 	std::cout << x_.size() << std::endl;
	  
	// Now see if the abscissa is out of range of the function
	
	if ( xalue < x_.at(0)) 
	  return b_.at(0);
	  
	if ( xalue > x_.at(n-1))  
	  return b_.at(n-1);
	  
	
  if (n == 1) 
    return 0;
	
  int k = 0; 
  if (n == 2)
    k = 0;    
  else { 
	  // Do a binary search to find which two spline control points the abscissa
	  // is between   
	
	  int i = 0;
	  int j = n;
	  
	  while (1)
	  {
		  k = (i+j)/2;
		  if (xalue < x_.at(k))
			  j = k;
		  else if (xalue > x_.at(k+1))
			  i = k;
		  else
			  break;
	  }   
	} 
	double dx = xalue - x_.at(k);
/*	std::cout << "==:: " << dx << " " << k << " " << x_.size()  << " " << std::flush;
	std::cout << d_.at(k) << " " << std::flush;
	std::cout << c_.at(k) << " " << std::flush;
	std::cout << b_.at(k) << " " << std::flush;
	std::cout << (b_.at(k) + dx * (2.0 * c_.at(k) + 3.0 * dx * d_.at(k))) << " " << std::flush;*/
	return (b_.at(k) + dx * ( 2.0 * c_.at(k) + 3.0 * dx * d_.at(k)));
	 
}


template <CurveMode::Mode mode>
double Curve<mode>::getSecondDerivative(double xValue) const  {

    
    int n = x_.size();
    
      
    // Now see if the abscissa is out of range of the function
    
    if ( xValue < x_.at(0)) 
      return b_.at(0);
      
    if ( xValue > x_.at(n-1))  
      return b_.at(n-1);
      
    
  if (n == 1) 
    return 0;
    
    int k = 0; 
  if (n == 2)
    k = 0;    
  else { 
      // Do a binary search to find which two spline control points the abscissa
      // is between   
    
      int i = 0;
      int j = n;
      
      while (1)
      {
          k = (i+j)/2;
          if (xValue < x_.at(k))
              j = k;
          else if (xValue > x_.at(k+1))
              i = k;
          else
              break;
      }   
    } 
    double dx = xValue - x_.at(k);
    return ( 2.0 * c_.at(k) + 6.0 * dx * d_.at(k) );
     
}


template <CurveMode::Mode mode>
std::ostream& operator<< (std::ostream& output, const Curve<mode>& c)
{
    for(unsigned i = 0; i < c.x_.size(); ++i)
        output << c.x_.at(i) << " ";
    output << std::endl;
    for(unsigned i = 0; i < c.y_.size(); ++i)
        output << c.y_.at(i) << " ";
    output << std::endl;
    for (std::vector<double>::const_iterator it = c.b_.begin(); it < c.b_.end(); ++it)
        output << *it << " ";
    output << std::endl;
    for (std::vector<double>::const_iterator it = c.c_.begin(); it < c.c_.end(); ++it)
        output << *it << " ";
    output << std::endl;
    for (std::vector<double>::const_iterator it = c.d_.begin(); it < c.d_.end(); ++it)
        output << *it << " ";
    output << std::endl;
    return output;
}

 
