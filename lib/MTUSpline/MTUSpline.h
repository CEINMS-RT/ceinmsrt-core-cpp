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

#ifndef Spline_h
#define Spline_h

#include <vector>
#include "SplineBasisFunction.h"
#include "MTUSplineBase.h"
#include <algorithm>

template<int dim>
class MTUSpline;

template<int dim>
class MTUSplineMult;

/**
 * MTUSpline class for one dim.
 * This class owns getter that have non const return use with caution
 * @TODO change the getter with non const return.
 */
template<>
class MTUSpline<1>: public MTUSplineBase
{
public:
	MTUSpline(const double a, const double b, const int n);
	MTUSpline(const std::vector<double>& a, const std::vector<double>& b,
			const std::vector<int>& n);
	void computeFewCoefficients(const std::vector<double>& y,
			std::vector<double>::iterator fromWhereInY);
	void computeCoefficients(const std::vector<double>& y,
			std::vector<double>::iterator fromWhereInY);

	/**
	 * Compute lmt.
	 */
	double getValue(const double x) const;

	/**
	 * Compute Ma.
	 */
	double getFirstDerivative(const double x);

	inline const int getDim() const
	{
		return 1;
	}

	// For recursive function
	inline const std::vector<double>& getCoefficients() const
	{
		return c_;
	}
	inline std::vector<double>& getCoefficients()
	{
		return c_;
	}
	inline const std::vector<double>& getCoefficientsSplineSecondPhase() const
	{
		return c_;
	} // Dim 1 have no secondPhase so we just return his coefficients
	inline std::vector<double>& getCoefficientsSplineSecondPhase()
	{
		return c_;
	}
	inline const MTUSpline<1>& getSplineFirstPhase() const
	{
		return *this;
	} // Dim 1 have no firstPhase so we just return himself
	inline MTUSpline<1>& getSplineFirstPhase()
	{
		return *this;
	}
	inline const std::vector<int>& getN()
	{
		int n = n_;
		vect_n_.push_back(n);
		return vect_n_;
	}
	inline void setCoefficients(const std::vector<double> coeff)
	{
		c_ = coeff;
	}

	template<int dim>
	friend class MTUSpline;
private:
	double a_;	//!< Initial node
	double b_;	//!< Final node
	int n_;   	//!< Number of node
	double h_; 	//!< Distance between node

	void computeInterval(int& l, int& m, const double x) const;

	std::vector<double> c_; //!< Coefficients

	std::vector<int> vect_n_; //!< For recursive function. Do not use !.

};

/**
 * MTUSpline Template class for dim superior to one.
 * This class owns getter that have non const return use with caution
 * @TODO change the getter with non const return.
 */
template<int dim>
class MTUSpline: public MTUSplineBase
{
public:
	MTUSpline(const std::vector<double>& a, const std::vector<double>& b,
			const std::vector<int>& n);
	void computeCoefficients(std::vector<double>& y,
			std::vector<double>::iterator fromWhereInY);

	/**
	 * Compute lmt.
	 */
	double getValue(const std::vector<double>& x);
	double getValue(const std::vector<double>& x) const;

	/**
	 * Compute Ma.
	 */
	double getFirstDerivative(const std::vector<double>& x, const int dimDerivative);

	inline const std::vector<double>& getCoefficients() const
	{
		return c_;
	}
	inline std::vector<double>& getCoefficients()
	{
		return c_;
	}
	inline void setCoefficients(const std::vector<double> coeff)
	{
		c_ = coeff;
	}

	inline const MTUSpline<dim - 1>& getSplineFirstPhase() const
	{
		return splineFirstPhase_;
	}
	inline MTUSpline<dim - 1>& getSplineFirstPhase()
	{
		return splineFirstPhase_;
	}

	inline void setCoefficientsSplineFirstPhase(const std::vector<double> coeff)
	{
		splineFirstPhase_.getCoefficients() = coeff;
	}

	inline const std::vector<double>& getCoefficientsSplineSecondPhase() const
	{
		return splineSecondPhase_.getCoefficients();
	}
	inline std::vector<double>& getCoefficientsSplineSecondPhase()
	{
		return splineSecondPhase_.getCoefficients();
	}

	inline void setCoefficientsSplineSecondPhase( const std::vector<double> coeff)
	{
		splineSecondPhase_.getCoefficients() = coeff;
	}

	inline const std::vector<int>& getN() const
	{
		return n_;
	}
	inline const std::vector<double>& getA() const
	{
		return a_;
	}
	inline const std::vector<double>& getB() const
	{
		return b_;
	}
	inline const int getDim() const
	{
		return dim;
	}

	friend class MTUSpline<dim + 1> ;

protected:
	std::vector<double> a_; //!< Initial node
	std::vector<double> b_; //!< Final node
	std::vector<int> n_; 	//!< Number of node
	std::vector<double> h_;	//!< Distance between node

	int sizeOfY_;
	bool checkValues(const std::vector<double>& x) const;
	void rectifiedValues(std::vector<double>& x);
	void computeInterval(std::vector<int>& l, std::vector<int>& m, const std::vector<double>& x) const;
	MTUSpline<dim - 1> splineFirstPhase_;
	MTUSpline<1> splineSecondPhase_;
	std::vector<double> c_;
};

#endif
