// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
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

#include "MTUSplineBase.h"


class UseException: public exception
{
  virtual const char* what() const throw()
  {
    return "Nothing to do here, something went wrong or does'nt have quality code.";
  }
} useException;

MTUSplineBase::MTUSplineBase()
{}

MTUSplineBase::~MTUSplineBase()
{}

double MTUSplineBase::getValue(const std::vector<double>& x) const
{
	//throw useException;
	return 0;
}

double MTUSplineBase::getFirstDerivative(const std::vector<double>& x, const int dimDerivative)
{
	//throw useException;
	return 0;
}

void MTUSplineBase::computeCoefficients(std::vector<double>& y, std::vector<double>::iterator fromWhereInY)
{
	throw useException;
}

bool MTUSplineBase::checkValues(const std::vector<double>& x) const
{
	//throw useException;
	return false;
}

void MTUSplineBase::computeInterval(std::vector<int>& l, std::vector<int>& m, const std::vector<double>& x) const
{
	//throw useException;
}

const std::vector<double>& MTUSplineBase::getCoefficients() const
{
	std::vector<double> a_;
	//throw useException;
	return a_;
}

std::vector<double>& MTUSplineBase::getCoefficients()
{
	std::vector<double> a_;
	//throw useException;
	return a_;
}

void MTUSplineBase::setCoefficients(const std::vector<double> coeff)
{
	//throw useException;
}

const MTUSplineBase& MTUSplineBase::getSplineFirstPhase() const
{
	//throw useException;
	MTUSplineBase temp;
	return temp;
}

MTUSplineBase& MTUSplineBase::getSplineFirstPhase()
{
	//throw useException;
	MTUSplineBase temp;
	return temp;
}

void MTUSplineBase::setCoefficientsSplineFirstPhase(const std::vector<double> coeff)
{
	//throw useException;
}

const std::vector<double>& MTUSplineBase::getCoefficientsSplineSecondPhase() const
{
	std::vector<double> a_;
	//throw useException;
	return a_;
}

std::vector<double>& MTUSplineBase::getCoefficientsSplineSecondPhase()
{
	std::vector<double> a_;
	//throw useException;
	return a_;
}

const std::vector<int>& MTUSplineBase::getN() const
{
	std::vector<int> n_;
	//throw useException;
	return n_;
}



