// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

#include <math.h>
#include <float.h>
#include <time.h>
#include <iostream>
using std::cout;
using std::endl;
#include <vector>
using std::vector;
#include <string>
using std::string;

//#define LOG_SIMULATED_ANNEALING
//#define DEBUG

template<typename TorquesComputationT>
SumMinObjectiveFunction_singleF<TorquesComputationT>::SumMinObjectiveFunction_singleF(
	TorquesComputationT& torquesComputation, double epsilon,
	unsigned int nEpsilon) :
	torquesComputation_(torquesComputation), epsilon_(epsilon), nEpsilon_(
		nEpsilon), epsilonBase_(epsilon), nEpsilonBase_(nEpsilon)
{

	torquesComputation_.resizeTorquesVector(torques_);
	torquesComputation_.resizePenaltiesVector(penalties_);
	torquesComputation_.setInverseTorques(inverseTorques_);
	torquesComputation_.getDofsToCalibrateIndexList(dofsIndexListToCalibrate_);
	torquesComputation_.getLmtTimeStep(torqueTimeStep_);
	torquesComputation_.getInverseTorquesTimeStep(inverseTorquesTimeStep_);
	noTrials_ = torques_.size();
	noDoF_ = dofsIndexListToCalibrate_.size();

	_verbose = 1;

#ifdef DEBUG
	cout << "noDof " << noDoF_ << std::endl;
	cout << "dofs index list ";

	for (unsigned int i = 0; i < noDoF_; ++i)
		cout << dofsIndexListToCalibrate_.at(i) << " ";

	cout << std::endl;
	cout << "noTrials " << noTrials_ << std::endl;
	cout << "noFrames ";

	for (unsigned int i = 0; i < noTrials_; ++i)
		cout << torques_.at(i).at(0).size() << " ";

	cout << std::endl;
#endif

	torquesVariance_.resize(noTrials_);

	for (unsigned int t = 0; t < noTrials_; ++t)
		torquesVariance_.at(t).resize(noDoF_);

	// inizialize f_ and fOpt_ at the max value
	f_ = fOpt_ = DBL_MAX;
	fLatest_.resize(nEpsilon_);

	for (unsigned int a = 0; a < nEpsilon_; ++a)
		fLatest_.at(a) = DBL_MAX;

	computeVariance();
}

template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::reset()
{
	// inizialize f_ and fOpt_ at the max value
	f_ = fOpt_ = fp_ = DBL_MAX;

	for (unsigned int a = 0; a < nEpsilon_; ++a)
		fLatest_.at(a) = DBL_MAX;

	epsilon_ = epsilonBase_;
	nEpsilon_ = nEpsilonBase_;
}


template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::computeVariance()
{
	//torquesVariance is a matrix: first dimension is the trial, second dimension is the DoF
	// therefore torquestVariance.at(ct).at(k) means the variance in the moment of
	// k DoF for the trial ct
	unsigned int d;

	for (unsigned int dIndex = 0; dIndex < noDoF_; ++dIndex)
	{
		d = dofsIndexListToCalibrate_.at(dIndex);

		for (unsigned int t = 0; t < noTrials_; ++t)
		{
			int startSample = 0; // in Fopt we evaluate the model after 50 samples. Let's do the same for the variance as well
			/*if (inverseTorquesTimeStep_.at(t).size() > 200)
				startSample = 50;
			else
				startSample = 10;*/

			int noDataRowsTorque = inverseTorques_.at(t).at(d).size() - startSample; // let's compute the variance for the samples we use in evalFp()
			double sumTorque = 0.;

			for (int r = startSample; r < noDataRowsTorque + startSample; ++r) //we start from startSample
				sumTorque += inverseTorques_.at(t).at(d).at(r);


			double averageTorque = sumTorque / noDataRowsTorque;
			torquesVariance_.at(t).at(dIndex) = 0.;
			double epTorque = 0.;

			for (int r = startSample; r < noDataRowsTorque + startSample; ++r)
			{
				torquesVariance_.at(t).at(dIndex) += (inverseTorques_.at(t).at(
					d).at(r) - averageTorque)
					* (inverseTorques_.at(t).at(d).at(r) - averageTorque);
				epTorque += (inverseTorques_.at(t).at(d).at(r) - averageTorque);
			}

			// calculate variance
			torquesVariance_.at(t).at(dIndex) = torquesVariance_.at(t).at(
				dIndex) / (noDataRowsTorque - 1);

			if (torquesVariance_.at(t).at(dIndex) == 0)
				torquesVariance_.at(t).at(dIndex) = 1;
		}
	}
}

template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::evalFp()
{

	// get the torques
	torquesComputation_.computeTorquesAndPenalties(torques_, penalties_);
	// now I eval fp which is an array
	// each member is the fp for a different dof
	// and is the difference between computed and experimental torques
	// for all the trials and data rows

	// reset fp_
	fp_ = 0;
	unsigned int d;

	// first dimension: the trial
	for (unsigned int t = 0; t < noTrials_; ++t)
	{
		double fp_dof = 0;

		// second dimension: the DoF
		for (unsigned int dIndex = 0; dIndex < noDoF_; ++dIndex)
		{
			d = dofsIndexListToCalibrate_.at(dIndex);
			double trialDifference = 0;
			double penaltyFactor = 0;

			int startSample = 0;
			/*if (inverseTorquesTimeStep_.at(t).size() > 200)
				startSample = 50;
			else
				startSample = 10;*/

			unsigned int cpt = startSample; // Added 50 to remove the effect of the fiber velocity effct

			for (std::vector<double>::const_iterator it = inverseTorquesTimeStep_.at(t).begin() + startSample; it < inverseTorquesTimeStep_.at(t).end(); it++)
			{
				const int& cpt2 = std::distance<std::vector<double>::const_iterator>(inverseTorquesTimeStep_.at(t).begin(), it);

				if (*it >= torqueTimeStep_.at(t).at(cpt))
				{
					trialDifference += ((torques_.at(t).at(d).at(cpt)
						- inverseTorques_.at(t).at(d).at(cpt2))
						* (torques_.at(t).at(d).at(cpt)
							- inverseTorques_.at(t).at(d).at(cpt2)));
					penaltyFactor += penalties_.at(t).at(cpt);
					cpt++;

					if (cpt > torqueTimeStep_.at(t).size() - 1)
						break;
				}
			}
			trialDifference = (trialDifference
				/ torquesVariance_.at(t).at(dIndex) + penaltyFactor)
				/ (inverseTorques_.at(t).at(d).size() - startSample); //we start from startSample
			fp_dof += trialDifference;
		}
		
		fp_ += fp_dof / noDoF_;
	}

	fp_ = fp_ / noTrials_;
}

// if (fp_ <= f_)

template<typename TorquesComputationT>
bool SumMinObjectiveFunction_singleF<TorquesComputationT>::isFacceptable()
{

	if (fp_ > f_)
		return false;

	return true;
}

template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::updateF()
{

	f_ = fp_;
}

//if (fp_ < fOpt_) {

template<typename TorquesComputationT>
bool SumMinObjectiveFunction_singleF<TorquesComputationT>::isFoptAcceptable()
{

	if (fp_ > fOpt_)
	{
		return false;
	}

	//#ifndef NOGUI
	//	SyncToolsCal::Shared::torqueCalMutex.lock();
	//	SyncToolsCal::Shared::torqueCal = torques_;
	//	SyncToolsCal::Shared::torqueCalMutex.unlock();
	//	SyncToolsCal::Shared::fOptMutex.lock();
	//	SyncToolsCal::Shared::fOpt = fp_;
	//	SyncToolsCal::Shared::fOptMutex.unlock();
	//#endif
	return true;
}

template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::updateFopt()
{

	fOpt_ = fp_;
}

template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::printFs()
{

	cout << "fp_: ";
	cout << fp_ << " ";
	cout << std::endl;
	cout << "f_: ";
	cout << f_ << " ";
	cout << std::endl;
	cout << "fOpt_: ";
	cout << fOpt_ << " ";
	cout << std::endl;
}

// computation of Metropolis criteria
// p = exp(f-fp)/T
// is now, for multiple DoF:
// p = exp(averageOnAllDoF(f-fp)/T)

template<typename TorquesComputationT>
double SumMinObjectiveFunction_singleF<TorquesComputationT>::computeMetropolisCriteria(
	const double t)
{
	double p = (f_ - fp_);

#ifdef LOG_SIMULATED_ANNEALING
	cout << "f_ " << f_ << " fp_ " << fp_ << std::endl;
#endif

	return exp(p / t);
}

// return TRUE when simulated annealing has
// converged
// if change in fopt < epsilon in the latest Nepsilon iteration

template<typename TorquesComputationT>
bool SumMinObjectiveFunction_singleF<TorquesComputationT>::terminate()
{
	// updated fLatest.at(0) with the latest computed value

	if (_verbose > 2)
	{
		cout << "TERMINATE: fOpt_ ";
		cout << fOpt_ << " ";
		cout << std::endl;
	}

	fLatest_.at(0) = f_;


	if (_verbose > 2)
	{

		for (unsigned int a = 0; a < nEpsilon_; ++a)
		{
			cout << "TERMINATE: fLatest_[" << a << "]: ";
			cout << fLatest_.at(a) << " ";
			cout << std::endl;
		}

	}

	// 	COUT << "epsilon_" << epsilon_ << std::endl << std::flush;
	for (unsigned int a = 0; a < nEpsilon_; ++a)
	{
		// we check that each DoF had a change in fOpt less than epsilon
		if (fabs(fLatest_.at(a) - fOpt_) > epsilon_)
		{

			if (_verbose > 1)
			{
				cout << "Return false" << std::endl;
				cout << "fLatest_.at(" << a << ") - fOpt_ = " << fLatest_.at(a)
					<< " - " << fOpt_ << " > " << epsilon_ << std::endl;
			}

			return false;
		}
	}

	if (_verbose > 1)
	{
		cout << "Return true! " << std::endl;
		cout << "fOpt_ = " << fOpt_ << std::endl;
	}

	return true;
}

template<typename TorquesComputationT>
void SumMinObjectiveFunction_singleF<TorquesComputationT>::updateFandFlatest()
{
	for (unsigned int a = 1; a < nEpsilon_; ++a)
		fLatest_.at(nEpsilon_ - a) = fLatest_.at(nEpsilon_ - a - 1);

	f_ = fOpt_;
}
