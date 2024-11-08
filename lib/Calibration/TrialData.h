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
#ifndef TrialData_h
#define TrialData_h

#include <CommonCEINMS.h>

#include <vector>
#include <string>
#include <iomanip> 

class TrialData
{
	public:

		void crop ( const double from, const double to );
		std::string id_; // OK
		int noMuscles_; // OK
		int noEmgSteps_; // OK
		std::vector< double > emgTimeSteps_;  // OK
		std::vector< std::vector < double > > emgData_; // OK
		int noLmtSteps_; // OK
		std::vector< double > lmtTimeSteps_; // OK
		std::vector< std::vector < double > > lmtData_; // OK
		int noDoF_; // OK
		std::vector<std::string> dofNames_; // OK
		// each ma has a different matrix
		std::vector< std::vector < std::vector < double > > > maData_; // OK
		unsigned int noTorqueSteps_;
		std::vector< std::vector < double > > torqueData_;
		std::vector< double > torqueTimeSteps_; // OK
		double _timeInit;

		inline void setVerbose ( const int& verbose )
		{
			_verbose = verbose;
		}

	protected:
		int _verbose;
};


#include "TrialData.cpp"

#endif
