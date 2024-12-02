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
