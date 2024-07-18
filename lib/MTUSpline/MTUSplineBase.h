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
