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



