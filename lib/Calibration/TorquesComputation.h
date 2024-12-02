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

#ifndef TorquesComputation_h
#define TorquesComputation_h

#include <CommonCEINMS.h>
#include <iostream>

#include <vector>
#include <string>
#include "NMSmodel.h"
#include "TrialData.h"

template <typename ComputationModeT, typename NMSmodelT>
class TorquesComputation;

template <typename ComputationModeT, typename NMSmodelT>
std::ostream& operator<< ( std::ostream& output, const TorquesComputation<ComputationModeT, NMSmodelT>& m );

template <typename ComputationModeT, typename NMSmodelT>
class TorquesComputation
{
	public:
		TorquesComputation(NMSmodelT& subject);
		TorquesComputation ( NMSmodelT& subject,
				const std::string& inputDataDirectory,
				const std::vector<std::string>& idTrials,
				const std::vector<std::string>& dofsToCalibrate );
		void resizeTorquesVector ( std::vector< std::vector< std::vector< double > > >& torques );
		void resizeMusclesVector ( std::vector< std::vector< std::vector< double > > >& musclesVector );
		void resizePenaltiesVector ( std::vector< std::vector< double > >& penalties );
		void computeTorquesAndPenalties ( std::vector< std::vector< std::vector< double > > >& torques,
				std::vector< std::vector< double> >& penalties );
		void setInverseTorques ( std::vector< std::vector< std::vector< double > > >& inverseTorques );
		void getDofsToCalibrateIndexList ( std::vector<unsigned int>& dofsIndexList );
		void getMusclesIndexFromDofs ( std::vector<unsigned int>& musclesIndexList );
		void getInverseTorquesTimeStep ( std::vector< std::vector< double > >& inverseTorquesTimeStep );
		void getLmtTimeStep ( std::vector< std::vector< double > >& lmtTimeStep );

		friend std::ostream& operator<< <> ( std::ostream& output, const TorquesComputation& m );

	protected:
		ComputationModeT computationMode_;
		NMSmodelT& subject_;
		std::vector<TrialData> trials_;
		std::vector<unsigned> musclesIndexList_;
		std::vector<std::string> dofsToCalibrate_;
};

#include "TorquesComputation.cpp"

#endif
