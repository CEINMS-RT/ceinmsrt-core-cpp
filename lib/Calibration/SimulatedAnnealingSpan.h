// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software, regulated by the license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots"
//

#ifndef SimulatedAnnealingSpan_h
#define SimulatedAnnealingSpan_h

#include <CommonCEINMS.h>

#include <string>
#include <vector>
#include <iostream>

#include "NMSmodel.h"
#include "TorquesComputation.h"
#include "simulatedAnnealing.hxx"
#include "ExecutionSimulatedAnnealing.h"

#include <ctime>

// This is implementation is from the paper
// Global Optimization of Statistical Functions with Simulated Annealing
// by W. L. Goffe, G. D. Ferrier and J. Rogers
// please refer to the papers for the meaning of the variables

const int PARAM = 0;
const int FOPT 	= 1;
const int END 	= 3;


template <typename ParametersT, typename ObjectiveFunctionT,typename TorquesComputationT,typename NMSmodelT>
class SimulatedAnnealingSpan;



template <typename ParametersT, typename ObjectiveFunctionT,typename TorquesComputationT,typename NMSmodelT>
class SimulatedAnnealingSpan {

public:
	SimulatedAnnealingSpan(
			NMSmodelT& subject,
			std::vector<std::string> dofsList,
			const std::string& configurationFile,
			TorquesComputationT& torquesComputation);

	void optimizeSpan(int argc, char *argv[]);
	
	inline void setVerbose ( const int& verbose )
	{
		_verbose = verbose;
	}

	inline void setGui ( const bool& gui )
	{
		_gui = gui;
	}

   //friend std::ostream& operator<< <> (std::ostream& output, const SimulatedAnnealing& sa);
	

private:
//	void SimulatedAnnealingSpan();
	void createBaseParameters();
	void sendResult();
	void checkBounds(int k);
		
		// data from xml file for simulated annealing configuration
// 	std::auto_ptr<SimulatedAnnealingType> annealingPointer_;
		// input output with the subject (set/get)
	ParametersT parameters_;
		// this is implementing how we compute fp
	ObjectiveFunctionT objectiveFunction_;
	


	std::vector<double> x_;
	std::vector<double> upperBounds_;
	std::vector<double> lowerBounds_;
	std::vector<double> xOpt_;
	std::vector<double> xp_;
	std::vector<double> v_;
	std::vector<int>    noAccepted_;
	int                 noParameters_;
	  
	double nt_;
	double ns_;
	double rt_;
	double t_;
	int maxNoEval_;
	bool terminateGen_;
	int _verbose;
	bool _gui;
	
	ExecutionSimulatedAnnealing _executionSimulatedAnnealing;
};

#include "SimulatedAnnealingSpan.cpp"

#endif
