// Copyright (c) 2015, Massimo Sartori, Monica Reggiani and Guillaume Durandau
// All rights reserved.

// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, 
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "MTUSpline.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>

#define PRECISION 6

//#define DEBUG
//#define LOG_SPLINE

template<int dim>
MTUSpline<dim>::MTUSpline(const std::vector<double>& a,
		const std::vector<double>& b, const std::vector<int>& n) :
		a_(a), b_(b), n_(n), splineFirstPhase_(a_, b_, n_), splineSecondPhase_(
				a_[dim - 1], b_[dim - 1], n_[dim - 1])
{
	for (int i = 0; i < dim; ++i)
	{
		h_.push_back((b_[i] - a_[i]) / n_[i]);
	}

#ifdef LOG_SPLINE  
	std::cout << " Creating Spline<" << dim << ">" << std::endl;
#endif

	int sizeOfCoeff = 1;
	for (int i = dim - 1; i >= 0; --i)
		sizeOfCoeff *= (n_[i] + 3);

	c_.resize(sizeOfCoeff);

	sizeOfY_ = 1;
	for (int i = dim - 1; i >= 0; --i)
		sizeOfY_ *= (n_[i] + 1);

#ifdef LOG_SPLINE
	std::cout << " a " << a_[dim-1] << " b " << b_[dim-1] << " n " << n_[dim-1] << " h " << h_[dim-1] << std::endl;
	std::cout << " number of Coeff " << sizeOfCoeff << std::endl;
	std::cout << " number of Y data: " << sizeOfY_ << std::endl << std::endl;
#endif 
}

template<int dim>
void MTUSpline<dim>::computeCoefficients(std::vector<double>& y,
		std::vector<double>::iterator fromWhereInY)
{
	// compute the number of value to be interpolated
	//std::vector<double>::iterator toWhereInY = fromWhereInY + sizeOfY_;


	// step 1: compute preCoefficients
	int numberOfPreCoeffs = (n_[dim - 1] + 1);
	for (int i = dim - 2; i >= 0; i--)
		numberOfPreCoeffs *= (n_[i] + 3);

	std::vector<double> preCoeffs;
	std::vector<double>::iterator posPreCoeffs;
	preCoeffs.reserve(numberOfPreCoeffs);

#ifdef LOG_SPLINE 
	std::cout << " Step 1: Dim: " << dim << " the number of precoefficients is: " << numberOfPreCoeffs << std::endl;
#endif

	std::vector<double>::iterator currentY = fromWhereInY;
	for (int i = 0; i <= n_[dim - 1]; ++i)
	{
		splineFirstPhase_.computeCoefficients(y, currentY);
		preCoeffs.insert(preCoeffs.end(),
				splineFirstPhase_.getCoefficients().begin(),
				splineFirstPhase_.getCoefficients().end());
		currentY += sizeOfY_ / (n_[dim - 1] + 1);
	}

// step 2: Now solve the spline interpolation problem
#ifdef LOG_SPLINE
	std::cout << "Beginning of step 2" << std::endl;
#endif     
	c_.clear();
	if (!c_.empty())
	{
		std::cout << " Should be empty!!!\n";
		exit(EXIT_FAILURE);
	}
	int sizeOfC = 1;
	for (int i = 0; i < dim; ++i)
		sizeOfC *= (n_[i] + 3);
	c_.resize(sizeOfC);

	int noInterpolatedDataFromPreCoeffs = numberOfPreCoeffs / (n_[dim - 1] + 1);
	std::vector<double> interpolatedDataFromPreCoeffs(
			noInterpolatedDataFromPreCoeffs);

	for (int i = 0; i < noInterpolatedDataFromPreCoeffs; ++i)
	{
		for (int j = 0; j < (n_[dim - 1] + 1); ++j)
		{
			interpolatedDataFromPreCoeffs[j] = preCoeffs[j
					* noInterpolatedDataFromPreCoeffs + i];

		}
		splineSecondPhase_.computeCoefficients(interpolatedDataFromPreCoeffs,
				interpolatedDataFromPreCoeffs.begin());
		for (int j = 0; j < n_[dim - 1] + 3; j++)
		{
			c_[j * (noInterpolatedDataFromPreCoeffs) + i] =
					splineSecondPhase_.getCoefficients()[j];
		}

	}
#ifdef LOG_SPLINE
	std::cout << "Spline<" << dim << ">'s " << c_.size() <<" coeffs\n";
	for (int i = 0; i < c_.size(); ++i)
	std::cout << c_[i] << std::endl;
	std::cout << std::endl;
#endif  

}

template<int dim>
bool MTUSpline<dim>::checkValues(const std::vector<double>& x) const
{
	for (int i = 0; i < dim; ++i)
		if ((x[i] < a_[i]) || (x[i] > b_[i]))
		{
//			cout << std::fixed << std::setprecision(PRECISION) << x[i] << " < " << a_[i] << "\t" << x[i] << " > " << b_[i] << endl;
			return false;
		}
	return true;
}

template<int dim>
void MTUSpline<dim>::rectifiedValues(std::vector<double>& x)
{
	for (int i = 0; i < dim; ++i)
		if (x[i] < a_[i])
			x[i] = a_[i];
		else if (x[i] > b_[i])
			x[i] = b_[i];
}

template<int dim>
void MTUSpline<dim>::computeInterval(std::vector<int>& l, std::vector<int>& m,
		const std::vector<double>& x) const
{
	for (int i = 0; i < dim; ++i)
	{
		l[i] = floor((x[i] - a_[i]) / h_[i]);
		m[i] = std::min(l[i] + 3, n_[i] + 2);
	}
}

template<int dim>
double MTUSpline<dim>::getValue(const std::vector<double>& x) const
{
	std::vector<int> l(dim);
	std::vector<int> m(dim);
	if (!checkValues(x))
	{
		std::cout << "Values x are out of boundaries\n";
		exit(EXIT_FAILURE);
	}
	computeInterval(l, m, x);

	std::vector<int> numberOfIterations(dim);
	for (int i = 0; i < dim; ++i)
	{
		numberOfIterations[i] = (m[i] - l[i] + 1);
	}

	int totIterations = 1;
	for (int i = 0; i < dim; ++i)
	{
		totIterations *= numberOfIterations[i];
	}

	std::vector<int> index(dim);
	double evaluatedValue = 0;
	for (int soFar = 0; soFar < totIterations; ++soFar)
	{
		double tot = 0;
		double mul = totIterations;
		for (int i = dim - 1; i > 0; --i)
		{
			mul = mul / numberOfIterations[i];
			index[i] = (soFar - tot) / mul;
			tot += index[i] * mul;
		}
		index[0] = soFar % numberOfIterations[0];

		mul = 1;
		int cIndex = 0;
		for (int i = 0; i < dim; ++i)
		{
			cIndex += (l[i] + index[i]) * mul;
			mul = mul * (n_[i] + 3);
		}
#ifdef LOG_SPLINE
		std::cout << "Multiply c_[" << cIndex << "]:" << c_[cIndex];
#endif
		double currentStep = 1;
		for (int i = 0; i < dim; ++i)
		{
#ifdef LOG_SPLINE
			std::cout << "\tx[" << i << "]:" << x[i] << "*" << "u[" << l[i]+index[i] << "]:" << SplineBasisFunction::getValue(x[i], l[i]+index[i], a_[i], h_[i]);
#endif
			currentStep *= SplineBasisFunction::getValue(x[i], l[i] + index[i],
					a_[i], h_[i]);
		}
#ifdef LOG_SPLINE
		std::cout << std::endl;
#endif
		evaluatedValue += c_[cIndex] * currentStep;
	}
	return evaluatedValue;
}

template<int dim>
double MTUSpline<dim>::getValue(const std::vector<double>& x)
{
	std::vector<int> l(dim);
	std::vector<int> m(dim);
//	if (!checkValues(x))
//	{
//		std::cout << "Values x are out of boundaries\n";
//		exit(EXIT_FAILURE);
//	}
	std::vector<double> rectX = x;
	rectifiedValues(rectX);
	computeInterval(l, m, rectX);

	std::vector<int> numberOfIterations(dim);
	for (int i = 0; i < dim; ++i)
	{
		numberOfIterations[i] = (m[i] - l[i] + 1);
	}

	int totIterations = 1;
	for (int i = 0; i < dim; ++i)
	{
		totIterations *= numberOfIterations[i];
	}

	std::vector<int> index(dim);
	double evaluatedValue = 0;
	for (int soFar = 0; soFar < totIterations; ++soFar)
	{
		double tot = 0;
		double mul = totIterations;
		for (int i = dim - 1; i > 0; --i)
		{
			mul = mul / numberOfIterations[i];
			index[i] = (soFar - tot) / mul;
			tot += index[i] * mul;
		}
		index[0] = soFar % numberOfIterations[0];

		mul = 1;
		int cIndex = 0;
		for (int i = 0; i < dim; ++i)
		{
			cIndex += (l[i] + index[i]) * mul;
			mul = mul * (n_[i] + 3);
		}
#ifdef LOG_SPLINE    
		std::cout << "Multiply c_[" << cIndex << "]:" << c_[cIndex];
#endif    
		double currentStep = 1;
		for (int i = 0; i < dim; ++i)
		{
#ifdef LOG_SPLINE    
			std::cout << "\tx[" << i << "]:" << x[i] << "*" << "u[" << l[i]+index[i] << "]:" << SplineBasisFunction::getValue(x[i], l[i]+index[i], a_[i], h_[i]);
#endif
			currentStep *= SplineBasisFunction::getValue(rectX[i], l[i] + index[i],
					a_[i], h_[i]);
		}
#ifdef LOG_SPLINE    
		std::cout << std::endl;
#endif
		evaluatedValue += c_[cIndex] * currentStep;
	}
	return evaluatedValue;
}

template<int dim>
double MTUSpline<dim>::getFirstDerivative(const std::vector<double>& x,
		const int dimDerivative)
{

	std::vector<int> l(dim);
	std::vector<int> m(dim);
//	if (!checkValues(x))
//	{
//		std::cout << "Values x are out of boundaries\n";
//		exit(EXIT_FAILURE);
//	}

	std::vector<double> rectX = x;
	rectifiedValues(rectX);

	computeInterval(l, m, rectX);

	std::vector<int> numberOfIterations(dim);
	for (int i = 0; i < dim; ++i)
	{
		numberOfIterations[i] = (m[i] - l[i] + 1);
	}

	int totIterations = 1;
	for (int i = 0; i < dim; ++i)
	{
		totIterations *= numberOfIterations[i];
	}

	std::vector<int> index(dim);
	double evaluatedValue = 0;
	for (int soFar = 0; soFar < totIterations; ++soFar)
	{
		double tot = 0;
		double mul = totIterations;
		for (int i = dim - 1; i > 0; --i)
		{
			mul = mul / numberOfIterations[i];
			index[i] = (soFar - tot) / mul;
			tot += index[i] * mul;
		}
		index[0] = soFar % numberOfIterations[0];

		mul = 1;
		int cIndex = 0;
		for (int i = 0; i < dim; ++i)
		{
			cIndex += (l[i] + index[i]) * mul;
			mul = mul * (n_[i] + 3);
		}
#ifdef LOG_SPLINE    
		std::cout << "Multiply c_[" << cIndex << "]:" << c_[cIndex];
#endif    
		double currentStep = 1;
		for (int i = 0; i < dimDerivative; ++i)
		{
#ifdef LOG_SPLINE    
			std::cout << "\tx[" << i << "]:" << x[i] << "*" << "u[" << l[i]+index[i] << "]:" << SplineBasisFunction::getValue(x[i], l[i]+index[i], a_[i], h_[i]);
#endif
			currentStep *= SplineBasisFunction::getValue(rectX[i], l[i] + index[i],
					a_[i], h_[i]);

		}

#ifdef LOG_SPLINE    
		std::cout << "\tdx/dim[" << dimDerivative << "]:" << x[dimDerivative] << "*" << "u[" << l[dimDerivative]+index[dimDerivative] << "]:" << SplineBasisFunction::getValue(x[dimDerivative], l[dimDerivative]+index[dimDerivative], a_[dimDerivative], h_[dimDerivative]);
#endif

		currentStep *= SplineBasisFunction::getFirstDerivative(rectX[dimDerivative],
				l[dimDerivative] + index[dimDerivative], a_[dimDerivative],
				h_[dimDerivative]);

		for (int i = dimDerivative + 1; i < dim; ++i)
		{
#ifdef LOG_SPLINE    
			std::cout << "\tx[" << i << "]:" << x[i] << "*" << "u[" << l[i]+index[i] << "]:" << SplineBasisFunction::getValue(x[i], l[i]+index[i], a_[i], h_[i]);
#endif
			currentStep *= SplineBasisFunction::getValue(rectX[i], l[i] + index[i],
					a_[i], h_[i]);

		}
#ifdef LOG_SPLINE    
		std::cout << std::endl;
#endif
		evaluatedValue += c_[cIndex] * currentStep;

	}

	return evaluatedValue;
}

/*************************************** Spline<1> ****************************************/

MTUSpline<1>::MTUSpline(const double a, const double b, const int n) :
		a_(a), b_(b), n_(n), h_((b - a) / n)
{
#ifdef LOG_SPLINE
	std::cout << " Creating Spline<1> n_" << n_ << std::endl;
#endif
	c_.empty();
	c_.resize(n_ + 3);

#ifdef LOG_SPLINE
	std::cout << std::endl << " Created Spline of dim 1 ";
	std::cout << " a " << a_ << " b " << b_ << " n " << n_ << " h " << h_ << std::endl;
	std::cout << " number of Coeff " << n_+3 << std::endl;
#endif
}

MTUSpline<1>::MTUSpline(const std::vector<double>& a,
		const std::vector<double>& b, const std::vector<int>& n) :
		a_(a[a.size() - 1]), b_(b[b.size() - 1]), n_(n[n.size() - 1]), h_(
				(b_ - a_) / n_)
{
#ifdef LOG_SPLINE
	std::cout << " Creating Spline<1> n_:" << n_ << std::endl;
#endif
	c_.empty();
	c_.resize(n_ + 3);

#ifdef LOG_SPLINE
	std::cout << std::endl << " Created Spline of dim 1 ";
	std::cout << " a " << a_ << " b " << b_ << " n " << n_ << " h " << h_ << std::endl;
	std::cout << " number of Coeff " << n_+3 << std::endl;
#endif

}

void MTUSpline<1>::computeCoefficients(const std::vector<double>& y,
		std::vector<double>::iterator fromWhereInY)
{
	std::vector<double>::iterator toWhereInY = fromWhereInY + (n_ + 1);
	std::vector<double> d;

	d.assign(fromWhereInY, toWhereInY);

	double alpha = (d[2] - 2 * d[1] + d[0]) / (h_ * h_);
	double beta = (d[n_] - 2 * d[n_ - 1] + d[n_ - 2]) / (h_ * h_);

	c_[1] = (d[0] - (alpha * h_ * h_) / 6) / 6;
	c_[n_ + 1] = (d[n_] - (beta * h_ * h_) / 6) / 6;

	// Thomas Algorithm to solve the linear equation
	std::vector<double> b(n_ - 1, 4);
	d[1] = d[1] - c_[1];
	d[n_ - 1] = d[n_ - 1] - c_[n_ + 1];

	double m;
	for (int k = 1; k <= (n_ - 2); ++k)
	{
		m = 1 / b[k - 1];
		b[k] = b[k] - m;
		d[k + 1] = d[k + 1] - m * d[k];
	}
	c_[n_] = d[n_ - 1] / b[n_ - 2];

	for (int k = n_ - 3; k >= 0; k--)
	{
		c_[k + 2] = (d[k + 1] - c_[k + 3]) / b[k];
	}
	c_[0] = (alpha * h_ * h_) / 6 + 2 * c_[1] - c_[2];
	c_[n_ + 2] = (beta * h_ * h_) / 6 + 2 * c_[n_ + 1] - c_[n_];

#ifdef LOG_SPLINE
	std::cout << "Spline<1>'s " << c_.size() << "coefficients" << std::endl;
	for (int i = 0; i < c_.size(); ++i)
		std::cout << c_[i] << std::endl;
#endif

}

void MTUSpline<1>::computeFewCoefficients(const std::vector<double>& y,
		std::vector<double>::iterator fromWhereInY)
{

	std::vector<double>::iterator toWhereInY = fromWhereInY + (n_ + 1);
	std::vector<double> d;
	d.assign(fromWhereInY, toWhereInY);

	double alpha = (d[2] - 2 * d[1] + d[0]) / (h_ * h_);
	double beta = (d[n_] - 2 * d[n_ - 1] + d[n_ - 2]) / (h_ * h_);



	c_[2] = (d[1] - c_[1] - c_[3]) / 4;
	c_[4] = beta * h_ / 6 + 2 * c_[3] - c_[2];
	c_[3] = (d[2] - c_[2] - c_[4]) / 4;
	c_[1] = (d[0] - c_[0] - c_[2]) / 4;
	c_[0] = (alpha * h_ * h_) / 6 - c_[2] + 2 * c_[1];

#ifdef DEBUG
	std::cout << "The coefficients are: " << std::endl;
	for (int i = 0; i <= n_+1; ++i)
	std::cout << c_[i] << std::endl;
#endif

}

void MTUSpline<1>::computeInterval(int& l, int& m, const double x) const
{
	l = floor((x - a_) / h_);
	m = std::min(l + 3, n_ + 2);
}

double MTUSpline<1>::getValue(const double x) const
{
	int l, m;
	double evaluatedValue = 0;

//	if ((x < a_) || (x > b_))
//	{
//		cout << std::fixed << std::setprecision(PRECISION) << x << " < " << a_ << "\t" << x << " > " << b_ << endl;
//		std::cout << "Value " << x << " out of boundaries\n";
//		exit(EXIT_FAILURE);
//	}

	double rectX = x;
	if (x < a_)
		rectX = a_;
	else if (x > b_)
		rectX = b_;

	computeInterval(l, m, rectX);
	for (int i = l; i <= m; ++i)
	{
		evaluatedValue += c_[i] * SplineBasisFunction::getValue(rectX, i, a_, h_);
	}

	return evaluatedValue;
}

double MTUSpline<1>::getFirstDerivative(const double x)
{
	int l, m;
	double evaluatedValue = 0;

//	if ((x < a_) || (x > b_))
//	{
//		std::cout << "Value " << x << " out of boundaries\n";
//		exit(EXIT_FAILURE);
//	}

	double rectX = x;
	if (x < a_)
		rectX = a_;
	else if (x > b_)
		rectX = b_;

	computeInterval(l, m, rectX);
	for (int i = l; i <= m; ++i)
		evaluatedValue += c_[i]
				* SplineBasisFunction::getFirstDerivative(rectX, i, a_, h_);

	return evaluatedValue;
}

template class MTUSpline<2> ;
template class MTUSpline<3> ;
template class MTUSpline<4> ;
template class MTUSpline<5> ;
template class MTUSpline<6> ;
template class MTUSpline<7> ;
template class MTUSpline<8> ;

