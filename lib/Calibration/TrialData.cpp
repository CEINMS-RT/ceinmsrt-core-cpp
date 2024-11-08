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
#include <iostream>
using std::cout;
using std::endl;
#include <vector>
using std::vector;
#include <stdlib.h>
#include <math.h>
#include <iomanip>

void TrialData::crop ( const double from, const double to )
{
	/*
		if (noLmtSteps_ != noTorqueSteps_)
		{
			cout << "ERROR in cropping trial\n";
			cout << noLmtSteps_ << " : " <<  noTorqueSteps_;
			exit(EXIT_FAILURE);
		}*/

//    if(to <= from || from < 0 || from > 1 || to < 0 || to > 1){
//        cout << "cropping failure\n";
//        return;
//    }

// 	COUT << from << " : " << to << std::endl;

	unsigned startSampleLmt = 0;
	unsigned stopSampleLmt = 0;

	bool firstPass = true;

	for ( vector<double>::const_iterator it = lmtTimeSteps_.begin();
			it != lmtTimeSteps_.end(); it++ )
	{
		if ( ( from + _timeInit ) <= *it && firstPass )
		{
			startSampleLmt = (unsigned) std::distance <
					vector<double>::const_iterator > (
							lmtTimeSteps_.begin(), it );
// 			COUT << "lmt: " << std::fixed << std::setprecision ( 20 )<< *it << std::endl;
			firstPass = false;
		}

		if ( ( to + _timeInit ) < *it )
		{
			stopSampleLmt = (unsigned) std::distance <
					vector<double>::const_iterator > (
							lmtTimeSteps_.begin(), it );
// 			COUT << "lmt: " << std::fixed << std::setprecision ( 20 ) << *it << std::endl;
			break;
		}
	}
	
// 	COUT << std::setprecision(15) << from + _timeInit << " ; " << to + _timeInit << std::endl << std::flush;

// 	if ( stopSampleLmt == 0 )
// 		stopSampleLmt = lmtTimeSteps_.size();
	
	if(stopSampleLmt <= startSampleLmt)
	{
		COUT << "In ik file for trial: " << id_ << " Cropping time is incorrect. Time boundary out of the time in the file." << std::endl << std::flush;
		exit(EXIT_FAILURE);
	}


//crop lmt data
	{
		vector<vector<double> >::const_iterator cropStart = lmtData_.begin()
				+ startSampleLmt;
		vector<vector<double> >::const_iterator cropEnd = lmtData_.begin()
				+ stopSampleLmt;
		vector<vector<double> > temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			cout << "cropping failure\n";
			return;
		}

		lmtData_ = temp;
	}

	{
		vector<double>::const_iterator cropStart = lmtTimeSteps_.begin()
				+ startSampleLmt;
		vector<double>::const_iterator cropEnd = lmtTimeSteps_.begin()
				+ stopSampleLmt;
		vector<double> temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			cout << "cropping failure\n";
			return;
		}

		lmtTimeSteps_ = temp;
	}

	noLmtSteps_ = (int) lmtTimeSteps_.size();

//crop torque data

	unsigned startSampleTorque = 0;
	unsigned stopSampleTorque = 0;

	firstPass = true;

	for ( vector<double>::const_iterator it = torqueTimeSteps_.begin();
			it != torqueTimeSteps_.end(); it++ )
	{
		if ( ( from + _timeInit ) <= *it && firstPass )
		{
			startSampleTorque = (unsigned) std::distance <
					vector<double>::const_iterator > (
							torqueTimeSteps_.begin(), it );

			if ( _verbose > 1 )
				std::cout << "lmt: " << std::fixed << std::setprecision ( 20 ) << *it << std::endl;

			firstPass = false;
		}

		if ( ( to + _timeInit ) < *it )
		{
			stopSampleTorque = (unsigned) std::distance <
					vector<double>::const_iterator > (
							torqueTimeSteps_.begin(), it );

			if ( _verbose > 1 )
				std::cout << "lmt: " << std::fixed << std::setprecision ( 20 ) << *it << std::endl;

			break;
		}
	}

// 	if ( stopSampleTorque == 0 )
// 		stopSampleTorque = torqueTimeSteps_.size();
	
	if(stopSampleTorque <= startSampleTorque)
	{
		COUT << "In id file for trial: " << id_ << " Cropping time is incorrect. Time boundary out of the time in the file." << std::endl << std::flush;
		exit(EXIT_FAILURE);
	}
/*	COUT << startSampleTorque << std::endl << std::flush;
	COUT << stopSampleTorque << std::endl << std::flush;
	COUT << torqueData_.at ( 0 ).size() << std::endl << std::flush;
	COUT << torqueData_.size() << std::endl << std::flush;
	COUT << noDoF_ << std::endl << std::flush;
*/
	for ( unsigned dofCt = 0; dofCt < (unsigned) noDoF_; ++dofCt )
	{
	  
// 		COUT << torqueData_.at ( dofCt ).size() << std::endl << std::flush;
		vector<double>::const_iterator cropStart = torqueData_.at ( dofCt ).begin()
				+ startSampleTorque;
		vector<double>::const_iterator cropEnd = torqueData_.at ( dofCt ).begin()
				+ stopSampleTorque;
				
//	COUT << torqueData_.at ( dofCt ).size() << std::endl << std::flush;
		vector<double> temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			cout << "cropping failure\n";
			return;
		}

		torqueData_.at ( dofCt ) = temp;
	}

	noTorqueSteps_ = (unsigned) torqueData_.at ( 0 ).size();

	{
		vector<double>::const_iterator cropStart = torqueTimeSteps_.begin()
				+ startSampleTorque;
		vector<double>::const_iterator cropEnd = torqueTimeSteps_.begin()
				+ stopSampleTorque;
		vector<double> temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			cout << "cropping failure\n";
			return;
		}

		torqueTimeSteps_ = temp;
	}

	//crop ma data
	for ( unsigned dofCt = 0; dofCt < (unsigned) noDoF_; ++dofCt )
	{
		vector<vector<double> >::const_iterator cropStart =
			maData_.at ( dofCt ).begin() + startSampleLmt;
		vector<vector<double> >::const_iterator cropEnd =
			maData_.at ( dofCt ).begin() + stopSampleLmt;
		vector<vector<double> > temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			cout << "cropping failure\n";
			return;
		}

		maData_.at ( dofCt ) = temp;
	}

}
