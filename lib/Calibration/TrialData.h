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
