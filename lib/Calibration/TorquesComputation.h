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
