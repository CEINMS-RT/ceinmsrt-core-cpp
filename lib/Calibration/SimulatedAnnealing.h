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

#ifndef SimulatedAnnealing_h
#define SimulatedAnnealing_h

#include "CommonCEINMS.h"
#include <string>
#include <vector>
#include <iostream>
#include <ctime>
//#include "NMSmodel.h"

#include "simulatedAnnealing.hxx"
#include "ExecutionSimulatedAnnealing.h"
// #include <boost/timer/timer.hpp>
#include "SyncToolsCal.h"

#ifdef UNIX
#include <sys/time.h>
#endif

// This is implementation is from the paper
// Global Optimization of Statistical Functions with Simulated Annealing
// by W. L. Goffe, G. D. Ferrier and J. Rogers
// please refer to the papers for the meaning of the variables


template <typename ParametersT, typename ObjectiveFunctionT, typename TorquesComputationT, typename NMSmodelT>
class SimulatedAnnealing;


template <typename ParametersT, typename ObjectiveFunctionT, typename TorquesComputationT, typename NMSmodelT>
std::ostream& operator<< ( std::ostream&,
		const SimulatedAnnealing < ParametersT,
		ObjectiveFunctionT,
		TorquesComputationT,
		NMSmodelT > & sa );


template <typename ParametersT, typename ObjectiveFunctionT, typename TorquesComputationT, typename NMSmodelT>
class SimulatedAnnealing
{

	public:
		SimulatedAnnealing ( NMSmodelT& subject,
				std::vector<std::string> dofsList,
				const ExecutionSimulatedAnnealing& executionSimulatedAnnealing,
				TorquesComputationT& torquesComputation );
		//constructor for hybrid annealing
		/*    SimulatedAnnealing(NMSmodel<Activation, Tendon >& subject,
		                       std::vector<std::string> dofsList,
		                       StaticTorquesComputation<ComputationMode, Activation, Tendon >& staticTorquesComputation);*/
		void optimize();
		friend std::ostream& operator<< <> ( std::ostream& output, const SimulatedAnnealing& sa );

		inline void setVerbose ( const int& verbose )
		{
			_verbose = verbose;
		}
		
		inline void setGui ( const bool& gui )
		{
			_gui = gui;
		}

		inline void setEndTimer(const double& endTimer)
		{
			_endTimer = endTimer;
		}

	protected:
		void checkBounds ( int k );

		// data from xml file for simulated annealing configuration
// 		std::auto_ptr<SimulatedAnnealingType> annealingPointer_;
		// input output with the subject (set/get)
		ParametersT parameters_;
		// this is implementing how we compute fp
		ObjectiveFunctionT objectiveFunction_;

		TorquesComputationT& torqueConput_;

		std::vector<double> x_;
		std::vector<double> upperBounds_;
		std::vector<double> lowerBounds_;
		std::vector<double> xOpt_;
		std::vector<double> xp_;
		std::vector<double> v_;
		std::vector<int>    noAccepted_;
		int                 noParameters_;
		
// 		ExecutionSimulatedAnnealing _executionSimulatedAnnealing;

//annealing parameters
		double              nt_;
		double              ns_;
		double              rt_;
		double              t_;
		int                 maxNoEval_;
		int _verbose;
		bool _gui;
		double _endTimer;
};

#include "SimulatedAnnealing.cpp"

#endif
