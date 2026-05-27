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

#ifndef MTUSPLINEBASE_H_
#define MTUSPLINEBASE_H_

#include <vector>
using std::vector;
#include <string>
using std::string;

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <sstream>
#include <iomanip>

using namespace std;

#include <vector>
#include "SplineBasisFunction.h"

/**
 * Base class for MTUSpline /!\ Do not use the method (protected by an exceptions)
 * Use only for downcasting.
 */
class MTUSplineBase
{
public:
	MTUSplineBase();
	virtual ~MTUSplineBase();
	virtual void computeCoefficients(std::vector<double>& y, std::vector<double>::iterator fromWhereInY);
	virtual double getValue(const std::vector<double>& x) const;
	virtual double getFirstDerivative(const std::vector<double>& x, const int dimDerivative);

	virtual const std::vector<double>& getCoefficients() const;
	virtual std::vector<double>& getCoefficients();
	virtual void setCoefficients(const std::vector<double> coeff);

	virtual const MTUSplineBase& getSplineFirstPhase() const;
	virtual MTUSplineBase& getSplineFirstPhase();
	virtual void setCoefficientsSplineFirstPhase(const std::vector<double> coeff);
	virtual const std::vector<double>& getCoefficientsSplineSecondPhase() const;
	virtual std::vector<double>& getCoefficientsSplineSecondPhase();

//	virtual inline void setCoefficientsSplineSecondPhase( const std::vector<double> coeff);
//
	virtual const std::vector<int>& getN() const;
//	virtual inline const std::vector<double>& getA() const;
//	virtual inline const std::vector<double>& getB() const;
//	virtual inline const int getDim() const;

protected:
	virtual bool checkValues(const std::vector<double>& x) const;
	virtual void computeInterval(std::vector<int>& l, std::vector<int>& m, const std::vector<double>& x) const;


};


#endif /* MTUSPLINEBASE_H_ */
