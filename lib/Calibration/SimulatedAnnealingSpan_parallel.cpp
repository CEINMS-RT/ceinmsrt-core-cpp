// This is part of
// NeuroMuscoloSkeletal Model Software (NMS)
// Copyright (C) 2010 David Lloyd Massimo Sartori Monica Reggiani
//
// ?? Licenza ??
//
// The authors may be contacted via:
// email: massimo.sartori@gmail.com monica.reggiani@gmail.com


#include "SimulatedAnnealingSpan_parallel.h"
#include "SetupDataStructure.h"
#include <iostream>
#include <fstream>
using std::cout;
using std::endl;

#include <math.h>
#include <time.h>
#include <vector>
using std::vector;
#include <string>
using std::string;
#include "mpi.h"
#include "random-gen.h"

//#define LOG_SIMULATED_ANNEALING

template < typename ParametersT, typename ObjectiveFunctionT,
				 typename TorquesComputationT, typename NMSmodelT >
SimulatedAnnealingSpan < ParametersT, ObjectiveFunctionT, TorquesComputationT,
											 NMSmodelT >::SimulatedAnnealingSpan ( NMSmodelT& subject,
													 std::vector<std::string> dofsList, const std::string& configurationFile,
													 TorquesComputationT& torquesComputation ) :
// 												 annealingPointer_ ( simulatedAnnealing ( configurationFile,
// 														 xml_schema::flags::dont_initialize ) ),
// 												 parameters_ ( subject, dofsList ),
// 												 objectiveFunction_ ( torquesComputation,  annealingPointer_->epsilon(), annealingPointer_->noEpsilon() )
_executionSimulatedAnnealing(configurationFile),
		parameters_ ( subject, dofsList ),
		objectiveFunction_ ( torquesComputation,  _executionSimulatedAnnealing.getEpsilon(), _executionSimulatedAnnealing.getNoEpsilon() )
{



	noParameters_ = parameters_.getNoParameters();

	noAccepted_.resize ( noParameters_ );

	x_.resize ( noParameters_ );
	parameters_.getStartingVectorParameters ( x_ );
	parameters_.setUpperLowerBounds ( upperBounds_, lowerBounds_ );

	xOpt_.resize ( noParameters_ );
	v_.resize ( noParameters_ );
	xp_.resize ( noParameters_ );
	noAccepted_.resize ( noParameters_ );
	total_NACP.resize ( noParameters_ );

	// we setup the data for the simulated annealing based on the configurationFile
		nt_ = _executionSimulatedAnnealing.getNT();
	ns_ = _executionSimulatedAnnealing.getNS();
	rt_ = _executionSimulatedAnnealing.getRT();
	t_ = _executionSimulatedAnnealing.getT();
	maxNoEval_ = _executionSimulatedAnnealing.getMaxNoEval();

	srand ( int ( time ( NULL ) / getpid() ) );
	rmarin ( ( rand() % 100 ) * 313.28, ( rand() % 100 ) * 300.81 );

}

template < typename ParametersT, typename ObjectiveFunctionT,
				 typename TorquesComputationT, typename NMSmodelT >
void SimulatedAnnealingSpan < ParametersT, ObjectiveFunctionT, TorquesComputationT,
		 NMSmodelT >::checkBounds ( int k )
{

	if ( ( xp_.at ( k ) < lowerBounds_.at ( k ) ) || ( xp_.at ( k ) > upperBounds_.at ( k ) ) )
		xp_.at ( k ) =
			lowerBounds_.at ( k )
			+ ( ( upperBounds_.at ( k ) - lowerBounds_.at ( k ) ) * rand()
					/ RAND_MAX );
}

template < typename ParametersT, typename ObjectiveFunctionT,
typename TorquesComputationT, typename NMSmodelT >
void SimulatedAnnealingSpan < ParametersT, ObjectiveFunctionT, TorquesComputationT,
		 NMSmodelT >::createBaseParameters()
{
	for ( int k = 0; k < noParameters_; ++k )
	{
		xp_.at ( k ) = lowerBounds_.at ( k ) + ( ( upperBounds_.at ( k ) - lowerBounds_.at ( k ) ) * ranmar() );
	}
}


template < typename ParametersT, typename ObjectiveFunctionT,
typename TorquesComputationT, typename NMSmodelT >
void SimulatedAnnealingSpan < ParametersT, ObjectiveFunctionT,
		 TorquesComputationT, NMSmodelT >::optimizeSpan ( int argc, char** argv )
{
	time_t now;
	time(&now);
	int RANK, ncpu, thiscpu;

	MPI_Comm_size ( MPI_COMM_WORLD, &ncpu );
	MPI_Comm_rank ( MPI_COMM_WORLD, &RANK );
	thiscpu = RANK + 1;

	int noEval = 0;
	
	objectiveFunction_.setVerbose(_verbose);

	for ( int i = 0; i < noParameters_; ++i )
		noAccepted_.at ( i ) = 0;

	createBaseParameters();
	parameters_.setVectorParameters ( xp_ );
// 	t_ = annealingPointer_->T();
	objectiveFunction_.reset();

	for ( int i = 0; i < noParameters_; ++i )
		v_.at ( i ) = ( upperBounds_.at ( i ) - lowerBounds_.at ( i ) ) / 2;

	// DO UNTIL convergence or maxNoEvaluation
	bool terminate = false;

	while ( ( !terminate ) && ( noEval < maxNoEval_ ) )
	{

		// DO Nt times
		for ( int i = 0; i < nt_; ++i )
		{

			// DO Ns times
			for ( int j = 0; j < ns_; ++j )
			{

				for ( int k = 0; k < noParameters_; ++k )
				{

					SyncToolsCal::Shared::endThreadMutex.lock();

					if ( SyncToolsCal::Shared::endThread )
					{
						SyncToolsCal::Shared::endThreadMutex.unlock();
						std::cout << "Quitting NMS Simulation" << std::endl;
						exit ( 0 );
					}

					SyncToolsCal::Shared::endThreadMutex.unlock();

					xp_ = x_;

					double factorForV = ( 2 * ranmar() ) - 1;
// 					double factorForV = ( 2. * rand() / static_cast<double> ( RAND_MAX ) - 1. );

					xp_.at ( k ) = x_.at ( k ) + v_.at ( k ) * factorForV;
					checkBounds ( k );

					parameters_.setVectorParameters ( xp_ );
					objectiveFunction_.evalFp();

					++noEval;

					// if f' < f then
					if ( objectiveFunction_.isFacceptable() )
					{
						x_ = xp_;

						objectiveFunction_.updateF();

						noAccepted_.at ( k ) ++;

						if ( objectiveFunction_.isFoptAcceptable() )
						{
							xOpt_ = xp_;

							objectiveFunction_.updateFopt();
						}
					}
					else
					{
						// IF f' > f THEN
						double p = objectiveFunction_.computeMetropolisCriteria ( t_ );
// 						double pp = ranmar();
						double pp = rand() / static_cast<double> ( RAND_MAX );

						if ( pp < p )
						{

							x_ = xp_;             // X = X'
							objectiveFunction_.updateF();// f_ = fp_;
							noAccepted_.at ( k ) ++;
						}
					}
				} // k
			} // j

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

//  			COUT << thiscpu << "; " << x_[0] << std::endl;
			
			SPAN_COMMUNICATE ( ncpu, thiscpu, c, noEval );
			
// 			COUT << thiscpu << "; " << x_[0] << std::endl;
			if ( _verbose > 1 )
				COUT << objectiveFunction_.getFopt() <<  endl;

		} // i

		terminate = objectiveFunction_.terminate();
		t_ = t_ * rt_;
		objectiveFunction_.updateFandFlatest();
		// reduce temperature



		// restart
		x_ = xOpt_;

	} /* end while */

	parameters_.setVectorParameters ( xOpt_ );
	objectiveFunction_.evalFp();

	if ( _verbose > 1 )
	{
		COUT << "Total evaluations:" << noEval << endl;

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
void SimulatedAnnealingSpan < ParametersT, ObjectiveFunctionT,
		 TorquesComputationT, NMSmodelT >::SPAN_COMMUNICATE ( int ncpu, int thiscpu,
				 std::vector<double> c, int noEval )
{

	/**** Local variables*/
	int size, I, j, k, best_cpu, new_best;
	int tag;
	double RATIO, best_fOpt_;
	MPI_Status status;

	/**** Determine fOpt_ and cpu that had best fOpt_*/
	best_cpu = 1;
	new_best = 1;
	tag = 1;

	double fOpt = objectiveFunction_.getFopt();
	double f = objectiveFunction_.getF();

	if ( _verbose > 1 )
		COUT << thiscpu << " : " << fOpt <<  endl;

	best_fOpt_ = 100000000;

	/**** Send data to cpu 1.*/
	if ( thiscpu > 2 )
	{
		MPI_Ssend ( &fOpt, 1, MPI_DOUBLE, 1, tag, MPI_COMM_WORLD );
		best_fOpt_ = fOpt;
		//cout<<fOpt_[0]<<endl;
	}
	/**** Receive data from all other cpu's.*/
	else
	{
		best_cpu = thiscpu;
		best_fOpt_ = fOpt;


		for ( j = 3; j <= ncpu; j++ )
		{
			MPI_Recv ( &fOpt, 1, MPI_DOUBLE, j - 1, tag, MPI_COMM_WORLD, &status );

			/****       Compare value of fOpt_ from cpu j to best_f_OPT*/
			if ( fOpt < best_fOpt_ )
			{
				best_cpu = j;
				best_fOpt_ = fOpt;

				//new_best = 1;
			}
		}

		fOpt = best_fOpt_;
		//system("pause");
	}

	//cout<<fOpt_[0]<<endl;

// 	cout<<thiscpu<< " 1" <<  endl;

	/**** Communicate which CPU was best*/
	tag = 2;

	if ( thiscpu == 2 )
	{
		for ( j = 3; j <= ncpu; j++ )
		{
			MPI_Ssend ( &best_cpu, 1, MPI_INT, j - 1, tag, MPI_COMM_WORLD );
		}
	}
	else
	{
		MPI_Recv ( &best_cpu, 1, MPI_INT, 1, tag, MPI_COMM_WORLD, &status );
	}

	/**** Adjust step length*/
	tag = 3;

	/**** Send nacp (number of accepted points) to best cpu*/
	if ( thiscpu != best_cpu )
	{
		MPI_Ssend ( noAccepted_.data(), noParameters_, MPI_INT, best_cpu - 1, tag, MPI_COMM_WORLD );

		for ( int i = 0; i < 1000000000; i++ )
			int t = i;

	}
	/**** Receive nacp from cpu's other than the best cpu*/
	else
	{
		/****    Initialize total_NACP*/
		for ( j = 0; j < noParameters_; j++ )
		{
			total_NACP[j] = noAccepted_[j];
		}

		for ( j = 2; j <= ncpu; j++ )
		{
			if ( j != best_cpu )
			{
				MPI_Recv ( noAccepted_.data(), noParameters_, MPI_INT, j - 1, tag, MPI_COMM_WORLD, &status );

				for ( k = 0; k < noParameters_; k++ )
					total_NACP[k] = total_NACP[k] + noAccepted_[k];
			}
		}

		/****    Update NACP*/


		for ( j = 0; j < noParameters_; j++ )
			noAccepted_[j] = total_NACP[j];

		/****    Adjust v_ so that approximately half of all evaluations are accepted.  ?????*/

		///*
		double sumratio = 0;

		for ( I = 0; I < noParameters_; I++ )

		{
			RATIO = ( double ) noAccepted_[I] / ns_;
			sumratio = sumratio + RATIO;

			//cout<< "indice " <<I <<"ratio " << RATIO<<endl;
			if ( RATIO > 0.6 )
				v_[I] = v_[I] * ( 1.0 + c[I] * ( RATIO - 0.6 ) / 0.4 );

			else if ( RATIO < 0.4 )
				v_[I] = v_[I] / ( 1.0 + c[I] * ( ( 0.4 - RATIO ) / 0.4 ) );

			if ( v_[I] > ( upperBounds_[I] - lowerBounds_[I] ) )
				v_[I] = upperBounds_[I] - lowerBounds_[I];


		}

		//*/

	}

	/**** Update all cpus*/
	tag = 4;

	/**** Best cpu sends new fOpt_, F, VM, X, and XOPT to other cpu's*/
	if ( thiscpu == best_cpu )
	{
		for ( j = 2; j <= ncpu; j++ )
		{
			if ( j != best_cpu )
			{

				MPI_Ssend ( &fOpt, 1, MPI_DOUBLE, j - 1, tag, MPI_COMM_WORLD );
				MPI_Ssend ( &f, 1, MPI_DOUBLE, j - 1, tag, MPI_COMM_WORLD );
				MPI_Ssend ( v_.data(), noParameters_, MPI_DOUBLE, j - 1, tag, MPI_COMM_WORLD );
				MPI_Ssend ( x_.data(), noParameters_, MPI_DOUBLE, j - 1, tag, MPI_COMM_WORLD );
				MPI_Ssend ( xOpt_.data(), noParameters_, MPI_DOUBLE, j - 1, tag, MPI_COMM_WORLD );
			}
		}

		MPI_Ssend ( xOpt_.data(), noParameters_, MPI_DOUBLE, 0, tag, MPI_COMM_WORLD );

		if ( _verbose > 2 )
			cout << fOpt << endl;

		MPI_Ssend ( &fOpt, 1, MPI_DOUBLE, 0, tag, MPI_COMM_WORLD );
	}
	/**** Receive fOpt_, F, VM, X, and XOPT from best cpu.*/
	else if ( thiscpu != best_cpu )
	{
		MPI_Recv ( &fOpt, 1, MPI_DOUBLE, best_cpu - 1, tag, MPI_COMM_WORLD, &status );
		MPI_Recv ( &f, 1, MPI_DOUBLE, best_cpu - 1, tag, MPI_COMM_WORLD, &status );
		MPI_Recv ( v_.data(), noParameters_, MPI_DOUBLE, best_cpu - 1, tag, MPI_COMM_WORLD, &status );
		MPI_Recv ( x_.data(), noParameters_, MPI_DOUBLE, best_cpu - 1, tag, MPI_COMM_WORLD, &status );
		MPI_Recv ( xOpt_.data(), noParameters_, MPI_DOUBLE, best_cpu - 1, tag, MPI_COMM_WORLD, &status );
	}

	/**** Reset NACP*/

	//system("pause");
	
// 	objectiveFunction_.setFopt(fOpt);
// 	objectiveFunction_.setFp(f);

	for ( I = 0; I < noParameters_; I++ )
		noAccepted_[I] = 0;

}


template < typename ParametersT, typename ObjectiveFunctionT,
typename TorquesComputationT, typename NMSmodelT >
std::ostream& operator<< ( std::ostream& output, const SimulatedAnnealingSpan < ParametersT, ObjectiveFunctionT,
		TorquesComputationT, NMSmodelT > & sa )
{
	output << "NT:        " << sa.nt_ << endl;
	output << "NS:        " << sa.ns_ << endl;
	output << "RT:        " << sa.rt_ << endl;
	output << "T:         " << sa.t_  << endl;
	output << "MaxNoEval: " << sa.maxNoEval_ << endl;

	output << "X:  ";

	for ( unsigned int i = 0; i < sa.x_.size(); ++i )
		output << sa.x_.at ( i ) << " ";

	output << endl;

	output << "Upper Bounds:  ";

	for ( unsigned int i = 0; i < sa.upperBounds_.size(); ++i )
		output << sa.upperBounds_.at ( i ) << " ";

	output << endl;

	output << "Lower Bounds: ";

	for ( unsigned int i = 0; i < sa.lowerBounds_.size(); ++i )
		output << sa.lowerBounds_.at ( i ) << " ";

	output << endl;

	return output;
}
