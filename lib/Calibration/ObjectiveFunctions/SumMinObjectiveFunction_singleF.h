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

#ifndef SumMinObjectiveFunction_singleF_h
#define SumMinObjectiveFunction_singleF_h

#include <vector>
#include "SyncToolsCal.h"
#include <CommonCEINMS.h>
/**
 *
 * This is the basic objective function dealing with multiple DoF
 *
 * if you want to replicate and add new objective function
 * they must have an implementation of the method
 * that are called by SimulatedAnnealing class
 *
 */

template<typename TorquesComputationT>
class SumMinObjectiveFunction_singleF
{
	public:
		SumMinObjectiveFunction_singleF ( TorquesComputationT& torquesComputation,
				double epsilon,
				unsigned nEpsilon );
		void   evalFp();
		bool   isFacceptable();
		void   updateF();
		bool   isFoptAcceptable();
		void   updateFopt();
		bool   terminate();
		void   updateFandFlatest();
		void   printFs();
		void reset();
		double computeMetropolisCriteria ( const double t );
		
		inline const std::vector < std::vector < std::vector < double > > >& getTorque()
		{
			return torques_;
		}
		
		inline const double& getF()
		{
			return f_;
		}
		
		inline const double& getFopt()
		{
			return fOpt_;
		}
		
		inline const double& getFp()
		{
			return fp_;
		}
		
		inline void setFopt ( const double& fOpt )
		{
			fOpt_ = fOpt;
		}
		
		inline void setFp ( const double& fp )
		{
			fp_ = fp;
		}

		inline void setVerbose ( const int& verbose )
		{
			_verbose = verbose;
		}

	protected:

		void computeVariance();
		TorquesComputationT&                                   torquesComputation_;
		std::vector < std::vector < std::vector < double > > > torques_;
		std::vector < std::vector < double > >                 penalties_;
		std::vector < std::vector < std::vector < double > > > inverseTorques_;
		std::vector < std::vector < double > >                 torquesVariance_;

		unsigned noTrials_;
		unsigned noDoF_;
		double   fp_;
		double   f_;
		double   fOpt_;
		unsigned nEpsilon_;
		double   epsilon_;
		unsigned nEpsilonBase_;
		double   epsilonBase_;
		int _verbose;

		std::vector< std::vector< double > > inverseTorquesTimeStep_;
		std::vector< std::vector< double > > torqueTimeStep_;


		std::vector<double>   fLatest_;
		std::vector<unsigned> dofsIndexListToCalibrate_;

};

#include "SumMinObjectiveFunction_singleF.cpp"

#endif
