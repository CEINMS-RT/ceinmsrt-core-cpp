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

#include "SimulatedAnnealing.h"
#include "TorquesComputation.h"

#include <iostream>
using std::cout;
using std::endl;

#include <math.h>
#include <time.h>
#include <vector>
using std::vector;
#include <string>
#include <chrono>
#include <boost/concept_check.hpp>
using std::string;

//#define LOG_SIMULATED_ANNEALING

template < typename ParametersT, typename ObjectiveFunctionT,
				 typename TorquesComputationT, typename NMSmodelT >
SimulatedAnnealing < ParametersT, ObjectiveFunctionT, TorquesComputationT, NMSmodelT >::SimulatedAnnealing ( NMSmodelT& mySubject,
	vector<string> dofsList, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, TorquesComputationT& torquesComputation ) :
// 		_executionSimulatedAnnealing(configurationFile),
		parameters_ ( mySubject, dofsList ),
		objectiveFunction_ ( torquesComputation,  executionSimulatedAnnealing.getEpsilon(), executionSimulatedAnnealing.getNoEpsilon() ),
		torqueConput_ ( torquesComputation )
{

// 	objectiveFunction_ = new ObjectiveFunctionT ( torquesComputation,  _executionSimulatedAnnealing.getEpsilon(), _executionSimulatedAnnealing.getNoEpsilon() );
	noParameters_ = parameters_.getNoParameters();
	x_.resize ( noParameters_ );
	parameters_.getStartingVectorParameters ( x_ );
	parameters_.setUpperLowerBounds ( upperBounds_, lowerBounds_ );

	xOpt_.resize ( noParameters_ );
	v_.resize ( noParameters_ );

	for ( int i = 0; i < noParameters_; ++i )
		v_.at ( i ) = ( upperBounds_.at ( i ) - lowerBounds_.at ( i ) ) / 2;

	xp_.resize ( noParameters_ );
	noAccepted_.resize ( noParameters_ );
	
	SyncToolsCal::Shared::nbOfEvalMutex.lock();
	SyncToolsCal::Shared::nbOfEval = 0;
	SyncToolsCal::Shared::nbOfEvalMutex.unlock();
	
	SyncToolsCal::Shared::convergenceMutex.lock();
	SyncToolsCal::Shared::convergence = 0;
	SyncToolsCal::Shared::convergenceMutex.unlock();
	
// 	COUT << "_executionSimulatedAnnealing.getNoEpsilon() " << executionSimulatedAnnealing.getNoEpsilon() << std::endl;

	// we setup the data for the simulated annealing based on the configurationFile
	nt_ = executionSimulatedAnnealing.getNT();
	ns_ = executionSimulatedAnnealing.getNS();
	rt_ = executionSimulatedAnnealing.getRT();
	t_ = executionSimulatedAnnealing.getT();
	maxNoEval_ = executionSimulatedAnnealing.getMaxNoEval();

	
	SyncToolsCal::Shared::maxEval = maxNoEval_;
	SyncToolsCal::Shared::maxEvalReady.notify();
	
	SyncToolsCal::Shared::minConv = executionSimulatedAnnealing.getEpsilon();
	SyncToolsCal::Shared::minConvReady.notify();

	_verbose = 1;

	_gui = false;

//    std::cout << "x_.size(): " << x_.size() << std::endl;;

	torquesComputation.setTimeTorques ( SyncToolsCal::Shared::timeIKBase );
	torquesComputation.getInverseTorquesTimeStep ( SyncToolsCal::Shared::timeIDBase );
	torquesComputation.setInverseTorques ( SyncToolsCal::Shared::torqueBase );
	SyncToolsCal::Shared::torqueBaseReady.notify();
	SyncToolsCal::Shared::timeIDReady.notify();
	SyncToolsCal::Shared::timeIKReady.notify();
	SyncToolsCal::Shared::vecParamLB = lowerBounds_;
	SyncToolsCal::Shared::vecParamUB = upperBounds_;
	SyncToolsCal::Shared::vecParamLBReady.notify();
	SyncToolsCal::Shared::vecParamUBReady.notify();
	SyncToolsCal::Shared::vecParamBase = x_;
	SyncToolsCal::Shared::vecParamBaseReady.notify();
	SyncToolsCal::Shared::dofNames = dofsList;
	mySubject.getMuscleNames ( SyncToolsCal::Shared::musclesNames );
	SyncToolsCal::Shared::dofNamesSem.notify();
	SyncToolsCal::Shared::musclesNamesSem.notify();
	mySubject.getMusclesIndexFromDofs ( SyncToolsCal::Shared::musclesIndexToCalibrate, dofsList );
	SyncToolsCal::Shared::musclesIndexToCalibrateSem.notify();
	torquesComputation.getDofsToCalibrateIndexList ( SyncToolsCal::Shared::dofIndexToCalibrate );
	SyncToolsCal::Shared::dofIndexToCalibrateSem.notify();



//	vector<string> dofName;
//	mySubject.getDoFNames(dofName);
//
//	for(vector<unsigned int>::const_iterator it = SyncToolsCal::Shared::dofIndexToCalibrate.begin(); it < SyncToolsCal::Shared::dofIndexToCalibrate.end(); it++)
//	{
//		std::cout << *it << " ";
//	}
//	std::cout <<std::endl;
//
//	for (vector<string>::const_iterator it1 = dofName.begin(); it1 != dofName.end(); it1++)
//	{
//		std::cout << *it1 << " ";
//	}
//	std::cout << std::endl;

//	exit(0);
#ifdef UNIX
	timeval tv;
	gettimeofday ( &tv, NULL );
	// :TODO: questo 1. fa cagare come seed fisso
	srand ( tv.tv_usec );
#endif
#ifdef WIN32
	srand(GetTickCount());
#endif
}

template < typename ParametersT, typename ObjectiveFunctionT,
				 typename TorquesComputationT, typename NMSmodelT >
void SimulatedAnnealing < ParametersT, ObjectiveFunctionT, TorquesComputationT,
		 NMSmodelT >::checkBounds ( int k )
{
	//parameters_.checkConstraint(xp_);
	if ( ( xp_.at ( k ) < lowerBounds_.at ( k ) ) || ( xp_.at ( k ) > upperBounds_.at ( k ) ) )
		xp_.at ( k ) =
			lowerBounds_.at ( k )
			+ ( ( upperBounds_.at ( k ) - lowerBounds_.at ( k ) ) * rand()
					/ RAND_MAX );
}

template < typename ParametersT, typename ObjectiveFunctionT,
typename TorquesComputationT, typename NMSmodelT >
void SimulatedAnnealing < ParametersT, ObjectiveFunctionT, TorquesComputationT,
		 NMSmodelT >::optimize()
{
	time_t now;
	time ( &now );
	std::vector<std::vector<std::vector<double> > > torques;
	std::vector<std::vector<double> > penalties;
	torqueConput_.resizeTorquesVector ( torques );
	torqueConput_.resizePenaltiesVector ( penalties );

	auto timeStart = std::chrono::high_resolution_clock::now();

	int noEval = 0;
	
	SyncToolsCal::Shared::finishMutex.lock();
	SyncToolsCal::Shared::finish = false;
	SyncToolsCal::Shared::finishMutex.unlock();

	for ( int i = 0; i < noParameters_; ++i )
		noAccepted_.at ( i ) = 0;

	// DO UNTIL convergence or maxNoEvaluation
	bool terminate = false;
	bool firstPass = true;
	
	while ( ( !terminate ) && ( noEval < maxNoEval_ ) )
	{

		// DO Nt times
		for ( int i = 0; i < nt_; ++i )
		{

			// DO Ns times
			for ( int j = 0; j < ns_; ++j )
			{

				// DO i = 1, n
				for ( int k = 0; k < noParameters_; ++k )
				{

					SyncToolsCal::Shared::endThreadMutex.lock();

					if ( SyncToolsCal::Shared::endThread )
					{
						SyncToolsCal::Shared::endThreadMutex.unlock();
						COUT << "Quitting NMS Simulation" << std::endl;
						exit ( 0 );
					}

					SyncToolsCal::Shared::endThreadMutex.unlock();

					xp_ = x_;

					if(!firstPass)
					{
					double factorForV = ( 2. * rand() / static_cast<double> ( RAND_MAX ) - 1. );
					
					xp_.at ( k ) = x_.at ( k ) + v_.at ( k ) * factorForV;
					checkBounds ( k );
					}
					else
					{
						firstPass = false;
					}

					parameters_.setVectorParameters ( xp_ );
					objectiveFunction_.evalFp();
					
					++noEval;

					//objectiveFunction_.printFs();

					// if f' < f then
					if ( objectiveFunction_.isFacceptable() )
					{
						// X = X'
						x_ = xp_;
						// f = f'
						objectiveFunction_.updateF();  // f_ = fp_;
						// update statistics
						noAccepted_.at ( k ) ++;

 					//	if ( _verbose > 2 )
 						//	cout << "F is ACCEPTABLE\n";

						// if f' < fopt then
						//if (fp_ < fOpt_) {
						if ( objectiveFunction_.isFoptAcceptable() )
						{
							// Xopt = X'
							xOpt_ = xp_;

							torqueConput_.computeTorquesAndPenalties ( torques, penalties );

							SyncToolsCal::Shared::torqueCalMutex.lock();
							SyncToolsCal::Shared::torqueCal = torques;
							SyncToolsCal::Shared::torqueCalMutex.unlock();
							SyncToolsCal::Shared::vecParamCalMutex.lock();
							SyncToolsCal::Shared::vecParamCal = xp_;
							SyncToolsCal::Shared::vecParamCalMutex.unlock();
							
							SyncToolsCal::Shared::nbOfEvalMutex.lock();
							SyncToolsCal::Shared::nbOfEval = noEval;
							SyncToolsCal::Shared::nbOfEvalMutex.unlock();
							
							SyncToolsCal::Shared::convergenceMutex.lock();
							SyncToolsCal::Shared::convergence = fabs( objectiveFunction_.getF() - objectiveFunction_.getFopt());
							SyncToolsCal::Shared::convergenceMutex.unlock();
							// fOpt = f'
							objectiveFunction_.updateFopt();// fOpt_ = fp_;

							SyncToolsCal::Shared::fOptMutex.lock();
							SyncToolsCal::Shared::fOpt = objectiveFunction_.getFopt();
							SyncToolsCal::Shared::fOptMutex.unlock();

// 							if ( _verbose > 2 )
// 								cout << "Fopt is ACCEPTABLE\n";
						}
					}
					else
					{
						// IF f' > f THEN
						double p = objectiveFunction_.computeMetropolisCriteria ( t_ );
						double randomValue = rand() / static_cast<double> ( RAND_MAX );

						if ( randomValue < p )
						{
// 							if ( _verbose > 2 )
// 							{
// 								cout << "through Metropolis " << randomValue << "<" << p;
// 								cout << " is acceptable\n";
// 							}

							x_ = xp_;             // X = X'
							objectiveFunction_.updateF();// f_ = fp_;
							noAccepted_.at ( k ) ++;
						}
// 						else
// 						{
// 							if ( _verbose > 2 )
// 							{
// 								cout << "through Metropolis NOT ACCEPTABLE\n";
// 							}
// 						}

					}

				} // k

			} // j

			// Adjust V such that half of all trial are accepted

			vector<double> c;
			c.resize ( noParameters_ );

			for ( int h = 0; h < noParameters_; ++h )
				c.at ( h ) = 2.;

			for ( int h = 0; h < noParameters_; ++h )
			{
				double ratio = noAccepted_.at ( h ) / ns_;

				if ( ratio > 0.6 )
					v_.at ( h ) = v_.at ( h ) * ( 1 + c.at ( h ) * ( ratio - 0.6 ) / 0.4 );
				else if ( ratio < 0.4 )
					v_.at ( h ) = v_.at ( h ) / ( 1 + c.at ( h ) * ( ( 0.4 - ratio ) / 0.4 ) );

				if ( v_.at ( h ) > ( upperBounds_.at ( h ) - lowerBounds_.at ( h ) ) )
					v_.at ( h ) = upperBounds_.at ( h ) - lowerBounds_.at ( h );
			}

			for ( int h = 0; h < noParameters_; ++h )
				noAccepted_.at ( h ) = 0.;

			if ( _verbose > 1 )
				COUT << noEval << std::endl;
			if ( _verbose > 1 )
				COUT << objectiveFunction_.getFopt() << std::endl;
// 			
// 			COUT << objectiveFunction_.getFopt() << std::endl;

		} // i

		// terminate?
		terminate = objectiveFunction_.terminate();

		auto currentTime = std::chrono::high_resolution_clock::now();
		auto calibrationTime = std::chrono::duration_cast<std::chrono::minutes>(currentTime - timeStart);
		std::cout << calibrationTime.count() << std::endl;
		
		//COUT << "Time passed: " << calibrationTime.count() << " minutes" << std::endl;
		
		if (_endTimer > 0 && calibrationTime.count() > _endTimer)
		{
			std::cout << "Set timer limit crossed" << std::endl;
			terminate = true;
		}

		// reduce temperature
		t_ = t_ * rt_;

		if ( _verbose > 2 )
			cout << "Temperature reduced at: " << t_ <<std::endl;

		objectiveFunction_.updateFandFlatest();
		// restart
		x_ = xOpt_;

	} /* end while */
	
	//COUT << "End of the Simulated Annealing" << std::endl;
	
	SyncToolsCal::Shared::finishMutex.lock();
	SyncToolsCal::Shared::finish = true;
	SyncToolsCal::Shared::finishMutex.unlock();

	parameters_.setVectorParameters ( xOpt_ );
// 	objectiveFunction_.evalFp();

	if ( _verbose > 1 )
	{
		COUT << "Total evaluations:" << noEval <<std::endl;

		time_t final;
		time ( &final );
		double total = difftime ( final, now );
		int minutes = int ( total ) / 60;
		int seconds = int ( total ) % 60;
		int hours = minutes / 60;
		minutes = minutes % 60;
		COUT << "Hours: " << hours << " Minutes: " << minutes << " seconds: " << seconds << std::endl;
	}
}

template < typename ParametersT, typename ObjectiveFunctionT,
typename TorquesComputationT, typename NMSmodelT >
std::ostream& operator<< ( std::ostream& output,
		const SimulatedAnnealing < ParametersT, ObjectiveFunctionT,
		TorquesComputationT, NMSmodelT > & sa )
{
	output << "NT:        " << sa.nt_ <<std::endl;
	output << "NS:        " << sa.ns_ <<std::endl;
	output << "RT:        " << sa.rt_ <<std::endl;
	output << "T:         " << sa.t_ <<std::endl;
	output << "MaxNoEval: " << sa.maxNoEval_ <<std::endl;

	output << "X:  ";

	for ( unsigned int i = 0; i < sa.x_.size(); ++i )
		output << sa.x_.at ( i ) << " ";

	output <<std::endl;

	output << "Upper Bounds:  ";

	for ( unsigned int i = 0; i < sa.upperBounds_.size(); ++i )
		output << sa.upperBounds_.at ( i ) << " ";

	output <<std::endl;

	output << "Lower Bounds: ";

	for ( unsigned int i = 0; i < sa.lowerBounds_.size(); ++i )
		output << sa.lowerBounds_.at ( i ) << " ";

	output <<std::endl;

	return output;
}

