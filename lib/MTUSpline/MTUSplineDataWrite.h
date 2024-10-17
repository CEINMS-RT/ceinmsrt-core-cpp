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

#ifndef MTUSPLINEDATAWRITE_H_
#define MTUSPLINEDATAWRITE_H_

#ifdef USE_OPENSIM
#include <MTUSplineData.h>
#include "MTUSplineBase.h"
#include "MuscleAnalyseForSpline.h"
//#include <boost/shared_ptr.hpp>
#include <memory>
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <sstream>
#include <iomanip>

/**
 * Class for creating the MTU spline coefficients files.
 */
class MTUSplineDataWrite: public MTUSplineData
{
public:

	MTUSplineDataWrite() : MTUSplineData()
	{
	}

	MTUSplineDataWrite(const string& subjectName) : MTUSplineData(subjectName)
	{
	}

	MTUSplineDataWrite(const MuscleAnalyseForSpline& muscleAnalyse);
	MTUSplineDataWrite(const MuscleAnalyseForSpline& muscleAnalyse, const string& subjectName);

	virtual ~MTUSplineDataWrite()
	{

	}

	/**
	 * Compute coefficients of the spline by task (one task by groups of DOF).
	 */
	void computeTaskCoeffients();

	/**
	 * Write coefficients in a file (one file by task).
	 */
	void writeTaskCoefficients();

	void writeCoefficients(const string& CoeffFilename);
	void writeCoefficients(const string& CoeffFilename, const vector< std::shared_ptr<MTUSplineBase> >& spline);
protected:
	/**
	 * Recursive function for writing all the coefficients of the first phase spline
	 * and second phase spline. Recall himself for writing the first phase spline
	 * and second phase spline for the N-1 DOF
	 * @param outputDataFile Output file for the coefficients
	 * @param FirstPhase Spline of the first phase
	 * @param secondPhaseCoefficients Coefficients of the second phase spline (1D)
	 * @param cptRecur Recursive enumerator
	 */
	template<class T>
	void writeCoefficientsFirstPhase(ofstream& outputDataFile, const T& FirstPhase,
			const std::vector<double>& secondPhaseCoefficients, int& cptRecur);

	MuscleAnalyseForSpline muscleAnalyse_;
	vector< vector< std::shared_ptr<MTUSplineBase> > > splinesTaskVect_;
};


#endif
#endif /* MTUSPLINEDATAWRITE_H_ */
