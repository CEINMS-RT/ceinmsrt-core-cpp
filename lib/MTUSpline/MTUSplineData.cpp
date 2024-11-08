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

#include <sstream>
using std::stringstream;

#include <iostream>
using std::cout;
using std::endl;
#define _USE_MATH_DEFINES

#include <math.h>
#include <stdlib.h>
#include <fstream>
using std::ifstream;
using std::ofstream;

#include <iomanip>
#include "MTUSplineBase.h"
#include "MTUSplineData.h"

#define PRECISION 6

using namespace std;

/**
 * Class for dimension exception.
 */
/*class DimException: public exception
{
  virtual const char* what() const throw()
  {
    return "The dimension is bigger than 8 which is the maximum accepted.";
  }
} dimException;*/

/**
 * Class for use exception.
 *//*
class UseException: public exception
{
  virtual const char* what() const throw()
  {
    return "Nothing to do here, something went wrong or does'nt have quality code.";
  }
} useException2;*/


#ifdef LOG_WRITE
vector<double> globalcoeff;
vector<double> globalcoeffRead;
#endif

const int DIGIT_NUM = 8;
const int NUMBER_DIGIT_OUTPUT = 8;

inline double roundIt(double x, double n = DIGIT_NUM)
{
	return floor(x * pow(10.0, n) + 0.5) / pow(10.0, n);
}

#ifndef MTU_h
inline double radians(double d)
{
	return d * M_PI / 180;
}

inline double degrees(double r)
{
	return r * 180 / M_PI;
}
#endif

MTUSplineData::~MTUSplineData()
{
//	for (vector< MTUSplineBase* >::iterator it1 = splines_.begin(); it1 != splines_.end(); it1++)
//				delete *it1;
}

void MTUSplineData::computeSplineCoeff(const string& inputDataFilename)
{
#ifdef LOG
	cout << "Reading input data from: "
	<< inputDataFilename << endl;
#endif

	readInputData(inputDataFilename);

#ifdef LOG
	cout << "Read the following interpolation data:\n";
	displayInputData();
#endif

	// create the noMuscles_ splines
	for (int i = 0; i < noMuscles_; ++i)
	{
		std::shared_ptr<MTUSpline<N_DOF> > newSpline(new MTUSpline<N_DOF>(a_, b_, n_));
		splines_.push_back(newSpline);

	}

#ifdef LOG
	cout << "Created " << splines_.size() << " splines.\n";
#endif

	// now compute coefficients for each muscle
	for (int i = 0; i < noMuscles_; ++i)
	{
		vector<double> currentMuscle(y_[i]);
		splines_[i]->computeCoefficients(currentMuscle, currentMuscle.begin());
	}
}

void MTUSplineData::computeSplineCoeff(
		const vector<vector<double> >& lmtVectorMat, const vector<double> a,
		vector<double> b, vector<int> n, vector<string> DOFNameVect,
		vector<string> muscleNameVect, int numOfDim)
{
	a_ = a;
	b_ = b;
	n_ = n;
	noMuscles_ = muscleNameVect.size();
	muscleNames_ = muscleNameVect;
	dofName_ = DOFNameVect;
	y_ = lmtVectorMat;

	computeSplineCoeff(numOfDim);

}

vector< std::shared_ptr<MTUSplineBase> > MTUSplineData::computeSplineCoeff(int numOfDim)
{
	splines_.clear();
	for (int i = 0; i < noMuscles_; ++i)
	{
		switch(numOfDim)
		{
			case 1:
			{
				std::shared_ptr<MTUSpline<1> > newSpline(new MTUSpline<1>(a_[0], b_[0], n_[0]));
				vector<double> currentMuscle(y_[i]);
				newSpline->computeCoefficients(currentMuscle, currentMuscle.begin());
				splines_.push_back(newSpline);
				//splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 2:
			{
				std::shared_ptr<MTUSpline<2> > newSpline(new MTUSpline<2>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 3:
			{
				std::shared_ptr<MTUSpline<3> > newSpline(new MTUSpline<3>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 4:
			{
				std::shared_ptr<MTUSpline<4> > newSpline( new MTUSpline<4>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 5:
			{
				std::shared_ptr<MTUSpline<5> > newSpline(new MTUSpline<5>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 6:
			{
				std::shared_ptr<MTUSpline<6> > newSpline(new MTUSpline<6>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 7:
			{
				std::shared_ptr<MTUSpline<7> > newSpline( new MTUSpline<7>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			case 8:
			{
				std::shared_ptr<MTUSpline<8> > newSpline( new MTUSpline<8>(a_, b_, n_));
				splines_.push_back(newSpline);
				vector<double> currentMuscle(y_[i]);
				splines_[i].get()->computeCoefficients(currentMuscle, currentMuscle.begin());
				break;
			}
			//default:
				//throw dimException;
		}
	}
	return splines_;
}

vector< std::shared_ptr<MTUSplineBase> > MTUSplineData::createSplineDim(int numOfDim)
{
	splines_.clear();
	for (int i = 0; i < noMuscles_; ++i)
	{
		switch(numOfDim)
		{
			case 1:
			{
				std::shared_ptr<MTUSpline<1> > newSpline(new MTUSpline<1>(a_[0], b_[0], n_[0]));
				splines_.push_back(newSpline);
				break;
			}
			case 2:
			{
				std::shared_ptr<MTUSpline<2> > newSpline(new MTUSpline<2>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			case 3:
			{
				std::shared_ptr<MTUSpline<3> > newSpline(new MTUSpline<3>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			case 4:
			{
				std::shared_ptr<MTUSpline<4> > newSpline( new MTUSpline<4>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			case 5:
			{
				std::shared_ptr<MTUSpline<5> > newSpline(new MTUSpline<5>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			case 6:
			{
				std::shared_ptr<MTUSpline<6> > newSpline(new MTUSpline<6>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			case 7:
			{
				std::shared_ptr<MTUSpline<7> > newSpline( new MTUSpline<7>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			case 8:
			{
				std::shared_ptr<MTUSpline<8> > newSpline( new MTUSpline<8>(a_, b_, n_));
				splines_.push_back(newSpline);
				break;
			}
			//default:
				//throw dimException;
		}
	}
	return splines_;
}

void MTUSplineData::readInputData(const string& inputDataFilename)
{

	// --- Read DOFs
	ifstream inputDataFile(inputDataFilename.c_str());
	if (!inputDataFile.is_open())
	{
		cout << "ERROR: " << inputDataFilename << " could not be open\n";
		exit(EXIT_FAILURE);
	}
	for (int i = 0; i < N_DOF; ++i)
	{
		inputDataFile >> dofName_[i];
		inputDataFile >> std::fixed >> std::setprecision(PRECISION) >> a_[i];
		inputDataFile >> std::fixed >> std::setprecision(PRECISION) >> b_[i];
		inputDataFile >> n_[i];
	}

	string line;
	getline(inputDataFile, line, '\n');
	getline(inputDataFile, line, '\n');
	stringstream myStream(line);
	string nextMuscleName;
	// --- Read Interpolation Data
	// 1. first their names
	do
	{
		myStream >> nextMuscleName;
		muscleNames_.push_back(nextMuscleName);
	} while (!myStream.eof());

	noMuscles_ = muscleNames_.size();

	// 2. then their values for all the possible combination of DOFs values
	// 2a. create the matrix to store them
	noInputData_ = 1;
	for (int i = 0; i < N_DOF; ++i)
		noInputData_ *= (n_[i] + 1);
	for (int i = 0; i < noMuscles_; ++i)
		y_.push_back(vector<double>(noInputData_));

	// 2b. read the data
	for (int j = 0; j < noInputData_; ++j)
		for (int i = 0; i < noMuscles_; ++i)
		{
			inputDataFile >> y_[i][j];
		}
	inputDataFile.close();
}

void MTUSplineData::displayInputData()
{

	cout << "-- DOFs  \n";
	cout << "DofName\t a \t b \t n \t h \n";
	for (int i = N_DOF - 1; i >= 0; --i)
	{
		cout << dofName_[i] << "\t";
		cout << a_[i] << "\t";
		cout << b_[i] << "\t";
		cout << n_[i] << "\t";
		cout << (b_[i] - a_[i]) / n_[i] << endl;
	}

	cout << "-- Data \n";
	for (int i = N_DOF - 1; i >= 0; --i)
		cout << dofName_[i] << "\t";

	for (int i = 0; i < noMuscles_; ++i)
	{
		cout << muscleNames_[i] << "\t";
	}
	cout << endl;

	vector<int> index(N_DOF);
	double tot, mul;
	for (int soFar = 0; soFar < noInputData_; ++soFar)
	{
		tot = 0;
		mul = noInputData_;
		for (int i = N_DOF - 1; i > 0; --i)
		{
			mul = mul / (n_[i] + 1);
			index[i] = (soFar - tot) / mul;
			tot += index[i] * mul;
		}
		index[0] = soFar % (n_[0] + 1);
		for (int i = N_DOF - 1; i >= 0; --i)
		{
			cout << a_[i] + index[i] * (b_[i] - a_[i]) / n_[i] << "\t";
		}
		for (int j = 0; j < noMuscles_; ++j)
			cout << y_[j][soFar] << "\t";
		cout << endl;
	}
}

void MTUSplineData::readEvalAngles()
{
	string anglesFilename = evalDataDir_ + "/angles.in";
	ifstream anglesFile(anglesFilename.c_str());

	if (!anglesFile.is_open())
	{
		cout << "ERROR: " << anglesFilename << " could not be open\n";
		exit(EXIT_FAILURE);
	}

	anglesFile >> noEvalData_;
	for (int i = 0; i < noEvalData_; ++i)
		angles_.push_back(vector<double>(N_DOF));

	for (int i = 0; i < noEvalData_; ++i)
		for (int j = N_DOF - 1; j >= 0; --j)
		{
			anglesFile >> angles_[i][j];
		}

	anglesFile.close();

}

void MTUSplineData::readEvalAngles(const string& Filename, int dim)
{
	ifstream anglesFile(Filename.c_str());

		if (!anglesFile.is_open())
		{
			cout << "ERROR: " << Filename << " could not be open\n";
			exit(EXIT_FAILURE);
		}

		anglesFile >> noEvalData_;
		for (int i = 0; i < noEvalData_; ++i)
			angles_.push_back(vector<double>(dim));

		for (int i = 0; i < noEvalData_; ++i)
			for (int j = 0; j < dim; j++)
				anglesFile >> std::fixed >> std::setprecision(PRECISION) >> angles_[i][j];
		anglesFile.close();
}

void MTUSplineData::openOutputFile(ofstream& outputDataFile)
{

	if (!outputDataFile.is_open())
	{
		cout << "ERROR: outputDataFile could not be open\n";
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < noMuscles_; ++i)
		outputDataFile << muscleNames_[i] << "\t";
	outputDataFile << endl;

	for (int i = 0; i < noMuscles_; ++i)
		outputDataFile << "eval\t";
	outputDataFile << endl;
}

void MTUSplineData::openEvalFile(ifstream& evalDataFile)
{

	if (!evalDataFile.is_open())
	{
		cout << "ERROR: evalData File could not be open\n";
		exit(EXIT_FAILURE);
	}

	// check we have the same amount of data of angles
	int numRows;
	evalDataFile >> numRows;

	if (numRows != noEvalData_)
	{
		cout << "ERROR: we have " << noEvalData_ << " angles, but " << numRows
				<< " lines of data in input from the evalDataFile" << endl;
		exit(EXIT_FAILURE);
	}

	// check we have the same muscles
	string line;
	vector<string> muscleNames;
	getline(evalDataFile, line, '\n');
	getline(evalDataFile, line, '\n');
	stringstream myStream(line);
	string nextMuscle;

	do
	{
		myStream >> nextMuscle;
		muscleNames.push_back(nextMuscle);
	} while (!myStream.eof());

	if (muscleNames.size() != static_cast<unsigned int>(noMuscles_))
	{
		cout << "ERROR: we have " << noMuscles_ << " interpolated muscles, but "
				<< muscleNames.size() << " muscles in the evalDataFile" << endl;
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < noMuscles_; ++i)
	{
		if (muscleNames[i] != muscleNames_[i])
		{
			cout << "ERROR: the " << i << "-th muscle is " << muscleNames_[i]
					<< " in the interpolated Data while" << muscleNames[i]
					<< " in file evalData " << endl;
			exit(EXIT_FAILURE);
		}
	}

}

void MTUSplineData::evalLmt()
{

	// First open the inputDataFile
	string evalDataFilename = evalDataDir_ + "lmt.in";
	ifstream evalDataFile(evalDataFilename.c_str());

	openEvalFile(evalDataFile);

	// Then open the outputDataFile

	string outputDataFilename = evalDataDir_ + "lmt.out";
	ofstream outputDataFile(outputDataFilename.c_str());
	openOutputFile(outputDataFile);

	// now readData

	double nextValue;
	for (int j = 0; j < noEvalData_; ++j)
	{
		for (int i = 0; i < noMuscles_; ++i)
		{
			outputDataFile << std::fixed << std::setprecision(PRECISION)
					<< splines_[i]->getValue(angles_[j])
					<< "\t";
		}
		outputDataFile << endl;
	}
	outputDataFile.close();
}

template <class T>
void MTUSplineData::evalLmt(const string& outputDataFilename, T& splines)
{
	ofstream outputDataFile(outputDataFilename.c_str());

	outputDataFile << noEvalData_ << endl;

	for (int j = 0; j < noEvalData_; ++j)
	{
		for (int i = 0; i < noMuscles_; ++i)
		{
			cpt_++;
			outputDataFile << std::fixed << std::setprecision(PRECISION)
					<< splines[i]->getValue(angles_[j])
					<< "\t";
		}
		outputDataFile << endl;
	}
	outputDataFile.close();
}

void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<1> > >& splines)
{
	ofstream outputDataFile(outputDataFilename.c_str());

	outputDataFile << noEvalData_ << endl;

	for (int j = 0; j < noEvalData_; ++j)
	{
		for (int i = 0; i < noMuscles_; ++i)
		{
			cpt_++;
			outputDataFile << std::fixed << std::setprecision(PRECISION)
					<< splines[i]->getValue(angles_[j][0])
					<< "\t";
		}
		outputDataFile << endl;
	}
	outputDataFile.close();
}

void MTUSplineData::evalMa()
{
	// for all the degree of freedom
	for (int k = 0; k < N_DOF; ++k)
	{

		// First open the inputDataFile
		string evalDataFilename = evalDataDir_ + "ma" + dofName_[k] + ".in";
		ifstream evalDataFile(evalDataFilename.c_str());
		openEvalFile(evalDataFile);

		// Then open the outputDataFile

		string outputDataFilename = evalDataDir_ + "ma" + dofName_[k] + ".out";
		ofstream outputDataFile(outputDataFilename.c_str());
		openOutputFile(outputDataFile);

		double nextValue;
		for (int j = 0; j < noEvalData_; ++j)
		{
			for (int i = 0; i < noMuscles_; ++i)
			{
				outputDataFile << std::fixed << std::setprecision(PRECISION)
						<< -splines_[i]->getFirstDerivative(angles_[j], k) << "\t";
			}
			outputDataFile << endl;
		}
		outputDataFile.close();
	}
}

template <class T>
void MTUSplineData::evalMa(const string& outputDataFilename, T& splines, int dim)
{
	ofstream outputDataFile(outputDataFilename.c_str());

	outputDataFile << noEvalData_ << endl;
	for (int j = 0; j < noEvalData_; ++j)
	{
		for (int i = 0; i < noMuscles_; ++i)
		{
			cpt_++;
			outputDataFile << std::fixed << std::setprecision(PRECISION)
					<<-splines[i]->getFirstDerivative(angles_[j], dim) << "\t";

		}
		outputDataFile << endl;
	}
	outputDataFile.close();
}

void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<1> > >& splines, int dim)
{
	ofstream outputDataFile(outputDataFilename.c_str());

	outputDataFile << noEvalData_ << endl;

	for (int j = 0; j < noEvalData_; ++j)
	{
		for (int i = 0; i < noMuscles_; ++i)
		{
			cpt_++;
			outputDataFile << std::fixed << std::setprecision(PRECISION)<< -splines[i]->getFirstDerivative(angles_[j][0]) << "\t";
		}
		outputDataFile << endl;
	}
	outputDataFile.close();
}


template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<2> > >& splines, int dim);
template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<3> > >& splines, int dim);
template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<4> > >& splines, int dim);
template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<5> > >& splines, int dim);
template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<6> > >& splines, int dim);
template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<7> > >& splines, int dim);
template void MTUSplineData::evalMa(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<8> > >& splines, int dim);

template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<2> > >& splines);
template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<3> > >& splines);
template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<4> > >& splines);
template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<5> > >& splines);
template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<6> > >& splines);
template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<7> > >& splines);
template void MTUSplineData::evalLmt(const string& outputDataFilename, vector<std::shared_ptr<MTUSpline<8> > >& splines);
