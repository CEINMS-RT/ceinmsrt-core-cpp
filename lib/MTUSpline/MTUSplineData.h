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
