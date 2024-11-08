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

#include "MTUSplineDataWrite.h"

#define PRECISION 6

#ifdef USE_OPENSIM

MTUSplineDataWrite::MTUSplineDataWrite(const MuscleAnalyseForSpline& muscleAnalyse): muscleAnalyse_(muscleAnalyse), MTUSplineData("Subject1")
{

}

MTUSplineDataWrite::MTUSplineDataWrite(const MuscleAnalyseForSpline& muscleAnalyse, const string& subjectName): muscleAnalyse_(muscleAnalyse), MTUSplineData(subjectName)
{

}

void MTUSplineDataWrite::computeTaskCoeffients()
{
	for (int i = 0; i < muscleAnalyse_.getTaskSize(); i++)
	{
		const MuscleAnalyseForSpline::Task& task =  muscleAnalyse_.getTask(i);
		noMuscles_ = task.uniqueMuscleList.size();
		muscleNames_ = vector<string>(task.uniqueMuscleList.begin(), task.uniqueMuscleList.end());
		dofName_ = vector<string>(task.uniqueDOFlist.begin(), task.uniqueDOFlist.end());
		y_ = task.lmtVectorMat;
		a_.clear();
		b_.clear();
		n_.clear();
		for (set<string>::const_iterator it = task.uniqueDOFlist.begin(); it != task.uniqueDOFlist.end(); it++)
		{
			a_.push_back(muscleAnalyse_.getMin(*it));
			b_.push_back(muscleAnalyse_.getMax(*it));
			n_.push_back(muscleAnalyse_.getnbOfStep());
		}
		splinesTaskVect_.push_back(computeSplineCoeff(task.uniqueDOFlist.size()));
	}
}

void MTUSplineDataWrite::writeTaskCoefficients()
{
//	cout << "Writing coefficients... " << endl;
	for (int i = 0; i < muscleAnalyse_.getTaskSize(); i++)
	{
		const MuscleAnalyseForSpline::Task& task =  muscleAnalyse_.getTask(i);
		noMuscles_ = task.uniqueMuscleList.size();
		muscleNames_ = vector<string>(task.uniqueMuscleList.begin(), task.uniqueMuscleList.end());		
		dofName_ = vector<string>(task.uniqueDOFlist.begin(), task.uniqueDOFlist.end());
		a_.clear();
		b_.clear();
		n_.clear();
		for (set<string>::const_iterator it = task.uniqueDOFlist.begin(); it != task.uniqueDOFlist.end(); it++)
		{
			a_.push_back(muscleAnalyse_.getMin(*it));
			b_.push_back(muscleAnalyse_.getMax(*it));
			n_.push_back(muscleAnalyse_.getnbOfStep());
		}
		stringstream temp_str;
		temp_str<<(i);
		string temp = temp_str.str();
		string filename(subjectName_ + "_Coefficients_" + temp + ".bin");
		writeCoefficients(filename, splinesTaskVect_[i]);
	}
//	cout << "Finish to Write coefficients" << endl;
}

void MTUSplineDataWrite::writeCoefficients(const string& CoeffFilename, const vector< std::shared_ptr<MTUSplineBase> >& spline)
{
	string outputCoeffFilename = "cfg/SplineCoeff/" + CoeffFilename;
	ofstream outputDataFile(outputCoeffFilename.c_str(), std::ofstream::binary);
	outputDataFile << "Name of the subject\t" << "Number of DOF" << "Numbers of Muscle" << endl;
	outputDataFile << subjectName_ << "\t" << dofName_.size() << "\t" << noMuscles_ << endl;
	outputDataFile << endl; //Blank line

	for (int i = 0; i < dofName_.size(); ++i)
	{
		outputDataFile << dofName_[i] << " ";
		outputDataFile << std::fixed << std::setprecision(PRECISION) << a_[i] << " ";
		outputDataFile << std::fixed << std::setprecision(PRECISION) << b_[i] << " ";
		outputDataFile << n_[i];
		outputDataFile << endl;
	}

	outputDataFile << endl; //Blank line

	for (vector<string>::iterator it = muscleNames_.begin(); it != muscleNames_.end(); it++)
		outputDataFile << *it << "\t";
	outputDataFile << endl;

	int cptRecur;
	outputDataFile << "Muscle number\t" << "Coefficients\t" << "First Phase Coefficients\t" << "Second Phase Coefficients\t" << "so on ..." << endl;
	for (int i = 0; i < noMuscles_; ++i)
	{
		outputDataFile << i << "\t";
		const std::vector<double>& splineCoeff = spline[i]->getCoefficients(); // Risque de segfault
		for (std::vector<double>::const_iterator it = splineCoeff.begin();it != splineCoeff.end(); it++)
			outputDataFile << std::fixed << std::setprecision(20) << *it << "\t";
		cptRecur = N_DOF - 1;
		writeCoefficientsFirstPhase(outputDataFile,
				spline[i]->getSplineFirstPhase(),
				spline[i]->getCoefficientsSplineSecondPhase(), cptRecur);
		outputDataFile << endl;
	}
	outputDataFile.close();
}

void MTUSplineDataWrite::writeCoefficients(const string& CoeffFilename)
{
//	cout << "Writing coefficients... " << endl;
	string outputCoeffFilename = "cfg/SplineCoeff/" + CoeffFilename;
	ofstream outputDataFile(outputCoeffFilename.c_str(), std::ofstream::binary);
	outputDataFile << "Name of the subject\t" << "Number of DOF" << "Numbers of Muscle" << endl;
	outputDataFile << subjectName_ << "\t" << dofName_.size() << "\t" << noMuscles_ << endl;
	outputDataFile << endl; //Blank line

	for (int i = 0; i < dofName_.size(); ++i)
	{
		outputDataFile << dofName_[i] << " ";
		outputDataFile << std::fixed << std::setprecision(6) << a_[i] << " ";
		outputDataFile << std::fixed << std::setprecision(6) << b_[i] << " ";
		outputDataFile << n_[i];
		outputDataFile << endl;
	}

	outputDataFile << endl; //Blank line

	for (vector<string>::iterator it = muscleNames_.begin(); it != muscleNames_.end(); it++)
		outputDataFile << *it << "\t";
	outputDataFile << endl;

	int cptRecur;
	outputDataFile << "Muscle number\t" << "Coefficients\t" << "First Phase Coefficients\t" << "Second Phase Coefficients\t" << "so on ..." << endl;
	for (int i = 0; i < noMuscles_; ++i)
	{
		outputDataFile << i << "\t";
		const std::vector<double>& splineCoeff = splines_[i]->getCoefficients();
		for (std::vector<double>::const_iterator it = splineCoeff.begin();it != splineCoeff.end(); it++)
			outputDataFile << std::fixed << std::setprecision(20) << *it << "\t";
		cptRecur = N_DOF - 1;
		writeCoefficientsFirstPhase(outputDataFile,
				splines_[i]->getSplineFirstPhase(),
				splines_[i]->getCoefficientsSplineSecondPhase(), cptRecur);
		outputDataFile << endl;
	}
	outputDataFile.close();
//	cout << "Finish to Write coefficients" << endl;
}

template<class T>
void MTUSplineDataWrite::writeCoefficientsFirstPhase(ofstream& outputDataFile,
		const T& FirstPhase, const std::vector<double>& secondPhaseCoefficients,
		int& cptRecur)
{
	if (cptRecur == 1)
	{
		const std::vector<double>& splineCoeff = FirstPhase.getCoefficients();
		for (std::vector<double>::const_iterator it = splineCoeff.begin();it != splineCoeff.end(); it++)
			outputDataFile << std::fixed << std::setprecision(20) << *it<< "\t";
	}
	else
	{
		const std::vector<double>& splineCoeff = FirstPhase.getCoefficients();
		for (std::vector<double>::const_iterator it = splineCoeff.begin(); it != splineCoeff.end(); it++)
			outputDataFile << std::fixed << std::setprecision(20) << *it<< "\t";
		for (std::vector<double>::const_iterator it =secondPhaseCoefficients.begin(); it != secondPhaseCoefficients.end(); it++)
			outputDataFile << std::fixed << std::setprecision(20) << *it<< "\t";
		cptRecur--;
		writeCoefficientsFirstPhase(outputDataFile,FirstPhase.getSplineFirstPhase(),FirstPhase.getCoefficientsSplineSecondPhase(), cptRecur);
	}

}

#endif
