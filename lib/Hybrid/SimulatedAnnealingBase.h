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

#ifndef SimulatedAnnealingBase_h
#define SimulatedAnnealingBase_h

#include <string>
#include <vector>
#include <iostream>
#include <ctime>

#ifdef WIN32
#include <Windows.h>
#endif

#ifdef UNIX
#include <sys/time.h>
#endif

#include "HybridWeightings.h"

// This is implementation is from the paper
// Global Optimization of Statistical Functions with Simulated Annealing
// by W. L. Goffe, G. D. Ferrier and J. Rogers
// please refer to the papers for the meaning of the variables

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
class SimulatedAnnealingBase;


template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
std::ostream& operator<< (std::ostream&, const SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>& sa);

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
class SimulatedAnnealingBase {

public:
    
    SimulatedAnnealingBase () {}
    SimulatedAnnealingBase (double nt, double ns, double rt, double t, double maxNoEval);
    SimulatedAnnealingBase( NMSmodelT& mySubject, 
                            std::vector<std::string>& muscleNamesWithEMGtoTrack,
                            std::vector<std::string>& muscleNamesWithEMGtoPredict,
                            const HybridWeightings hybridParameters,
                            StaticComputationT& staticComputation,
                            const std::string performanceCriterion = ""); // optional because otherwise ErrorMinimizerOnlineCalibration doesn't work

    void optimize();
    void getXopt(std::vector<double>& xOpt) const;

	void setNT(double nt);
	void setNS(double ns);
	void setRT(double rt);
	void setT(double t);
	void setMaxNoEval(int maxNoEval);
	void setEpsilon(double epsilon);
	void setNEpsilon(unsigned nEpsilon);

	void init();

	void setBaseParam(std::vector<double> shapeFactorBase, std::vector<double>  tendonSlackLengthsBase,
		std::vector<double>  optimalFiberLengthsBase, std::vector<double>  groupMusclesBasedOnStrengthCoefficientsBase)
	{
		parameters_.setBaseParam(shapeFactorBase, tendonSlackLengthsBase, optimalFiberLengthsBase, groupMusclesBasedOnStrengthCoefficientsBase);
	}
    
//    friend class SimulatedAnnealingHybrid<NMSmodelT, Parameters, ObjectiveFunction, ComputationMode>;
    friend std::ostream& operator<< <> (std::ostream& output, const SimulatedAnnealingBase& sa);

protected:
    void checkBounds(int k);
    
    Parameters parameters_;
    ObjectiveFunction objectiveFunction_;
    
    
    std::vector<double> x_;
    std::vector<double> upperBounds_;
    std::vector<double> lowerBounds_;
    std::vector<double> xOpt_;
    std::vector<double> xp_;
    std::vector<double> v_;
    std::vector<int>    noAccepted_; 
    int                 noParameters_;
    
//annealing parameters    
    double              nt_;
    double              ns_;
    double              rt_;
    double              t_;
    int                 maxNoEval_;
	unsigned			nEpsilon_;
	double				epsilon_;
};

#include "SimulatedAnnealingBase.cpp"

#endif
