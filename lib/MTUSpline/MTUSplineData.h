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

#ifndef SplineData_h
#define SplineData_h

#include "MTUSpline.h"
#include "MTUSplineBase.h"

#include <vector>
using std::vector;
#include <string>
using std::string;

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <sstream>
#include <iomanip>
//#include <boost/shared_ptr.hpp>
#include <memory>

using namespace std;

const int N_DOF = 8;

//#define LOG_WRITE // Work only with testSlpineWrite () bwcause we do Read and Write

#ifdef LOG_WRITE
extern vector<double> globalcoeff;
extern vector<double> globalcoeffRead;
#endif

/**
 * Class for the creation and the handling of the MTU Spline.
 */
class MTUSplineData
{
public:

	MTUSplineData() :
			dofName_(N_DOF), a_(N_DOF), b_(N_DOF), n_(N_DOF), subjectName_("Subject1"), cpt_(0)
	{
	}

	/*
	 * Constructor that take the name of the subject for coefficients files.
	 */
	MTUSplineData(const string& subjectName) :
			dofName_(N_DOF), a_(N_DOF), b_(N_DOF), n_(N_DOF), subjectName_(subjectName.c_str()), cpt_(0)
	{
	}
	virtual ~MTUSplineData();

	/**
	 * Compute the spline for constant fixed spline (old version).
	 */
	void computeSplineCoeff(const string& inputDataFilename);

	/**
	 * Compute the spline for constant fixed spline (old version).
	 */
	void computeSplineCoeff(const vector<vector<double> >& lmtVectorMat, const vector<double> a, vector<double> b,
			vector<int> n, vector<string> DOFNameVect, vector<string> muscleNameVect, int numOfDim);

	/**
	 * Create the splines with the good dimension, call the computeCoefficients method of each splines
	 * and return it. The number of splines created depends of member noMuscles_.
	 * @param numOfDim the dimension of the spline.
	 * @return the vector of splines.
	 */
	vector< std::shared_ptr<MTUSplineBase> > computeSplineCoeff(int numOfDim);

	/**
	 * Create the spline with the good dimension and return it.
	 * The number of splines created depends of member noMuscles_.
	 * @param numOfDim the dimension of the spline.
	 * @return the vector of splines.
	 */
	vector< std::shared_ptr<MTUSplineBase> > createSplineDim(int numOfDim);

	inline void setEvalDataDir(const string& evalDataDir)
	{
		evalDataDir_ = evalDataDir;
	}

	/**
	 *  read the angles files (old version).
	 */
	void readEvalAngles();

	/**
	 *  compute lmt from angles file (old version).
	 */
	void evalLmt();

	/**
	 *  compute ma from angles file (old version).
	 */
	void evalMa();

protected:
	void readInputData(const string& inputDataFilename);
	void displayInputData();
	void openEvalFile(ifstream& evalDataFile);
	void openOutputFile(ofstream& outputDataFile);

	/**
	 * Read angles files and save it in a member variable.
	 */
	void readEvalAngles(const string& Filename, int dim);

	/**
	 * Template method for evaluating lmt.
	 */
	template <class T>
	void evalLmt(const string& outputDataFilename, T& splines);

	/*
	 * one dim method for evaluating lmt.
	 */
	void evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<1> > >& splines);

	/**
	 * Template method for evaluating ma.
	 */
	template <class T>
	void evalMa(const string& outputDataFilename, T& splines, int dim);

	/*
	 * one dim method for evaluating ma.
	 */
	void evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<1> > >& splines, int dim);

	long int cpt_;

	string subjectName_;

	// Interpolation Data
	vector<string> dofName_;
	vector<double> a_;
	vector<double> b_;
	vector<int> n_;
	vector<string> muscleNames_;
	int noMuscles_;
	int noInputData_;
	vector<vector<double> > y_;

	// Spline
	vector< std::shared_ptr<MTUSplineBase> > splines_;

	// EvalData
	string evalDataDir_;
	int noEvalData_;
	vector<vector<double> > angles_;

};


#endif
