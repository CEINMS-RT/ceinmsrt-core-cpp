// This is part of
// NeuroMuscoloSkeletal Model Software (NMS)
// Copyright (C) 2010 David Lloyd Massimo Sartori Monica Reggiani
//
// ?? Licenza ??
//
// The authors may be contacted via:
// email: massimo.sartori@gmail.com monica.reggiani@gmail.com

#include "SimulatedAnnealingSpan.h"
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

template<typename ParametersT, typename ObjectiveFunctionT,
		typename TorquesComputationT, typename NMSmodelT>
SimulatedAnnealingSpan<ParametersT, ObjectiveFunctionT, TorquesComputationT,
		NMSmodelT>::SimulatedAnnealingSpan(NMSmodelT& subject,
		std::vector<std::string> dofsList, const std::string& configurationFile,
		TorquesComputationT& torquesComputation) :
// 		annealingPointer_(simulatedAnnealing(configurationFile.c_str())), parameters_(
// 				subject, dofsList), objectiveFunction_(torquesComputation,
// 				annealingPointer_->epsilon(), annealingPointer_->noEpsilon())
				
		_executionSimulatedAnnealing(configurationFile),
		parameters_ ( subject, dofsList ),
		objectiveFunction_ ( torquesComputation,  _executionSimulatedAnnealing.getEpsilon(), _executionSimulatedAnnealing.getNoEpsilon() )
{

	noParameters_ = parameters_.getNoParameters();
	x_.resize(noParameters_);
	parameters_.getStartingVectorParameters(x_);
	parameters_.setUpperLowerBounds(upperBounds_, lowerBounds_);

	xOpt_.resize(noParameters_);
	v_.resize(noParameters_);
	xp_.resize(noParameters_);
	noAccepted_.resize(noParameters_);

	// we setup the data for the simulated annealing based on the configurationFile
		nt_ = _executionSimulatedAnnealing.getNT();
	ns_ = _executionSimulatedAnnealing.getNS();
	rt_ = _executionSimulatedAnnealing.getRT();
	t_ = _executionSimulatedAnnealing.getT();
	maxNoEval_ = _executionSimulatedAnnealing.getMaxNoEval();

	terminateGen_ = false;

	// Initialise the random number generator RANMAR
	srand(int(time(NULL) / getpid()));
	rmarin((rand() % 100) * 313.28, (rand() % 100) * 300.81);
}

template<typename ParametersT, typename ObjectiveFunctionT,
		typename TorquesComputationT, typename NMSmodelT>
void SimulatedAnnealingSpan<ParametersT, ObjectiveFunctionT, TorquesComputationT,
		NMSmodelT>::checkBounds(int k)
{

	if ((xp_.at(k) < lowerBounds_.at(k)) || (xp_.at(k) > upperBounds_.at(k)))
		xp_.at(k) = lowerBounds_.at(k) + ((upperBounds_.at(k) - lowerBounds_.at(k)) * rand() / RAND_MAX);
}

template<typename ParametersT, typename ObjectiveFunctionT,
		typename TorquesComputationT, typename NMSmodelT>
void SimulatedAnnealingSpan<ParametersT, ObjectiveFunctionT, TorquesComputationT,
		NMSmodelT>::createBaseParameters()
{
	for (int k = 0; k < noParameters_; ++k)
		xp_.at(k) = lowerBounds_.at(k) + ((upperBounds_.at(k) - lowerBounds_.at(k)) * ranmar());
}

template<typename ParametersT, typename ObjectiveFunctionT,
		typename TorquesComputationT, typename NMSmodelT>
void SimulatedAnnealingSpan<ParametersT, ObjectiveFunctionT,
		TorquesComputationT, NMSmodelT>::optimizeSpan(int argc, char** argv)
{
	time_t now;
	time(&now);
	while (!terminateGen_)
	{

		int noEval = 0;
		for (int i = 0; i < noParameters_; ++i)
			noAccepted_.at(i) = 0;

		createBaseParameters();
		parameters_.setVectorParameters(xp_);
// 		t_ = annealingPointer_->T();
		objectiveFunction_.reset();

		for (int i = 0; i < noParameters_; ++i)
			v_.at(i) = (upperBounds_.at(i) - lowerBounds_.at(i)) / 2;

		// DO UNTIL convergence or maxNoEvaluation
		bool terminate = false;

		while ((!terminate) && (noEval < maxNoEval_))
		{

			// DO Nt times
			for (int i = 0; i < nt_; ++i)
			{

				// DO Ns times
				for (int j = 0; j < ns_; ++j)
				{

					// cout<<"NS_ "<<j<<endl;
					// DO i = 1, n
					//for (int k = 0; k < noParameters_; ++k) {

					for (int k = 0; k < noParameters_; ++k)
					{

						SyncToolsCal::Shared::endThreadMutex.lock();
						if (SyncToolsCal::Shared::endThread)
						{
							SyncToolsCal::Shared::endThreadMutex.unlock();
							std::cout << "Quitting NMS Simulation" << std::endl;
							exit(0);
						}
						SyncToolsCal::Shared::endThreadMutex.unlock();

						xp_ = x_;

						double factorForV = (2 * ranmar()) - 1;

						xp_.at(k) = x_.at(k) + v_.at(k) * factorForV;
						checkBounds(k);

						parameters_.setVectorParameters(xp_);
						objectiveFunction_.evalFp();

						++noEval;

						// if f' < f then
						if (objectiveFunction_.isFacceptable())
						{
							x_ = xp_;

							objectiveFunction_.updateF();

							noAccepted_.at(k)++;

							if(objectiveFunction_.isFoptAcceptable())
							{
								xOpt_ = xp_;

								objectiveFunction_.updateFopt();
							}
						}
						else
						{ // IF f' > f THEN
							double p = objectiveFunction_.computeMetropolisCriteria(t_);
							double pp = ranmar();

							if (pp < p)
							{

								x_ = xp_;             // X = X'
								objectiveFunction_.updateF();// f_ = fp_;
								noAccepted_.at(k)++;
							}
						}
					} // k
				} // j

				vector<double> c;
				c.resize(noParameters_);
				for (int h = 0; h < noParameters_; ++h)
				c.at(h) = 2.;

				for (int h = 0; h < noParameters_; ++h)
				{
					double ratio = noAccepted_.at(h) / ns_;
					if (ratio > 0.6)
					v_.at(h) = v_.at(h) * ( 1 + c.at(h) * (ratio - 0.6) / 0.4 );
					else if (ratio < 0.4)
					v_.at(h) = v_.at(h) / (1 + c.at(h) * ((0.4 - ratio)/ 0.4));
					if (v_.at(h) > (upperBounds_.at(h) - lowerBounds_.at(h)))
					v_.at(h) = upperBounds_.at(h) - lowerBounds_.at(h);
				}

				for (int h = 0; h < noParameters_; ++h)
					noAccepted_.at(h) = 0.;

			} // i
			terminate = objectiveFunction_.terminate();
			t_ = t_ * rt_;
			objectiveFunction_.updateFandFlatest();
			// reduce temperature

			// restart
			x_ = xOpt_;

		} /* end while */

		parameters_.setVectorParameters(xOpt_);
		objectiveFunction_.evalFp();
		sendResult();
	}
	time_t final;
	time(&final);
	double total = difftime(final,now);
	int minutes = int(total) / 60;
	int seconds = int(total) % 60;
	int hours = minutes / 60;
	minutes = minutes % 60;
	COUT << "Hours: " << hours << " Minutes: " << minutes << " seconds: " << seconds << std::endl;
}

template<typename ParametersT, typename ObjectiveFunctionT,
		typename TorquesComputationT, typename NMSmodelT>
void SimulatedAnnealingSpan<ParametersT, ObjectiveFunctionT,
		TorquesComputationT, NMSmodelT>::sendResult()
{
	int RANK;
	MPI_Comm_rank(MPI_COMM_WORLD,&RANK);
//	std::cout << "send data: " << RANK << std::endl;

	MPI_Send(xOpt_.data(), xOpt_.size(), MPI::DOUBLE, 0,
			PARAM, MPI_COMM_WORLD);

	double fOpt = objectiveFunction_.getFopt();
	MPI_Send(&fOpt, 1, MPI::DOUBLE, 0,
			FOPT, MPI_COMM_WORLD);

	MPI_Recv(&terminateGen_, 1, MPI::BOOL, 0, END, MPI_COMM_WORLD,
	             MPI_STATUS_IGNORE);

}

template<typename ParametersT, typename ObjectiveFunctionT,
		typename TorquesComputationT, typename NMSmodelT>
std::ostream& operator<<(std::ostream& output,
		const SimulatedAnnealingSpan<ParametersT, ObjectiveFunctionT,
				TorquesComputationT, NMSmodelT>& sa)
{
	output << "NT:        " << sa.nt_ << endl;
	output << "NS:        " << sa.ns_ << endl;
	output << "RT:        " << sa.rt_ << endl;
	output << "T:         " << sa.t_ << endl;
	output << "MaxNoEval: " << sa.maxNoEval_ << endl;

	output << "X:  ";
	for (unsigned int i = 0; i < sa.x_.size(); ++i)
		output << sa.x_.at(i) << " ";
	output << endl;

	output << "Upper Bounds:  ";
	for (unsigned int i = 0; i < sa.upperBounds_.size(); ++i)
		output << sa.upperBounds_.at(i) << " ";
	output << endl;

	output << "Lower Bounds: ";
	for (unsigned int i = 0; i < sa.lowerBounds_.size(); ++i)
		output << sa.lowerBounds_.at(i) << " ";
	output << endl;

	return output;
}
