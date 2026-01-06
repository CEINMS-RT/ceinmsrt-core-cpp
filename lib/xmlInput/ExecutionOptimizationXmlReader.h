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

#ifndef ExecutionOptimizationXmlReader_h
#define ExecutionOptimizationXmlReader_h

#include <executionOptimization.hxx>
#include "HybridWeightings.h"
#include <cstring>
#include <vector>
#include <iostream>

class ExecutionOptimizationXmlReader
{
public:

	// >> Functions to get values

	ExecutionOptimizationXmlReader(const std::string& filename);
	~ExecutionOptimizationXmlReader();
	
	bool UseOfMuscleInTheLoop();
	bool UseOfHybrid();
	bool UseOfOnlineCalibration();
	bool UseOfMuscleParameter();
	bool UseSimulatedAnnealing();
	bool UseSimplex();
	bool UseErrorDrivenAdaptation();
	bool UseLM();

	// Muscle In The Loop Optimization
	std::vector<std::string> getMusclesList(); // Muscle list
	std::vector<std::string> getMusclesToOptimize(); // Muscle list to optimize
	std::vector<std::string> getDofsList(); // DOFs list
	std::string getOptimizationCriterion(); // EMG or Muscle forces
	double getWheightOptimizedMuscles(); // For the cost function
	double getWheightNonOptimizedMuscles(); // For the cost function
	double getReductionFactor(); // For the muscles to be optimized
	int getnCyclesRef(); // For taking the reference EMG
	int getnCyclesOptimization(); // How many steps between optimization

	// Hybrid Optimization
	std::vector<std::string> getHybridMuscleWithEMGToPredict(); // use by OnlineCalibration, MuscleParameter, Hybrid
	std::vector<std::string> getHybridMuscleWithEMG();// use by OnlineCalibration, Hybrid
	std::vector<double> getMuscleForceTreshold(); // use by MuscleParameter
	std::vector<double> getMuscleLengthTreshold();// use by MuscleParameter
	HybridWeightings getHybridWeightings();// use by MuscleParameter, Hybrid
	std::string getPerformanceCriterion(); // use by Hybrid
	std::vector<std::string> getHybridDOFsOptimized(); // use by Hybrid

	// Simulated Annealing
	double getNoEpsilon();
	double getRt();
	double getT();
	int getNS();
	int getNT();
	double getEpsilon();
	int getMaxNoEval();
	int getBufferSize(); // use by OnlineCalibration

protected:

	// >> Local Variables
	void readXml();

	std::auto_ptr<OptimizationType> executionPointer_;
	bool useOfMuscleInTheLoop_;
	bool useOfHybrid_;
	bool useOfOnlineCalibration_;
	bool useOfMuscleParameter_;
	bool simulatedAnealing_;
	bool lm_;
	bool simplex_;

	// Muscle In The Loop Optimization
	std::vector<std::string> musclesList_;
	std::vector<std::string> musclesToOptimize_;
	std::vector<std::string> dofsList_;
	std::string optimizationCriterion_;
	double wheightOptimizedMuscles_;
	double wheightNonOptimizedMuscles_;
	double reductionFactor_;
	int nCyclesRef_;
	int nCyclesOptimization_;

	// Hybrid Optimization
	std::vector<std::string> hybridMuscleWithEMGToPredict_;
	std::vector<std::string> hybridMuscleWithEMG_;
	HybridWeightings hybridWeightings_;
	std::string performanceCriterion_;
	std::vector<std::string> hybridDOFsOptimized_;
	std::vector<double> muscleForceTreshold_;
	std::vector<double> muscleLengthTreshold_;

	// Simulated Annealing
	double noEpsilon_;
	double rt_;
	double t_;
	int ns_;
	int nt_;
	int bufferSize_;
	double epsilon_;
	int maxNoEval_;

};

#endif