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
