// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox". 
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software, 
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS. 
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621, 
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com 

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
