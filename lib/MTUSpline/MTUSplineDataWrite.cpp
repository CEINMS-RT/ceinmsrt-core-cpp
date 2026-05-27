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
