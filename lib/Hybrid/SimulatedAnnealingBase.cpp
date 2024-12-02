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

#include <iostream>
using std::cout;
using std::endl;

#include <math.h>
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <cstdlib>
#include "HybridWeightings.h"

//#define LOG_SIMULATED_ANNEALING
/*
template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::
SimulatedAnnealingBase (double nt, double ns, double rt, double t, double maxNoEval):
nt_(nt), ns_(ns), rt_(rt), t_(t), maxNoEval_(maxNoEval) {
srand(1.);
}*/

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::SimulatedAnnealingBase(
	NMSmodelT& mySubject,
	vector<string>& muscleNamesWithEMGtoTrack,
	vector<string>& muscleNamesWithEMGtoPredict,
	const HybridWeightings hybridParameters,
	StaticComputationT& staticComputation,
	const std::string performanceCriterion) :
	parameters_(mySubject, muscleNamesWithEMGtoTrack, muscleNamesWithEMGtoPredict),
	objectiveFunction_(staticComputation, 1e-4, 8, hybridParameters, performanceCriterion)
{
	nt_ = 5;
	ns_ = 20;
	rt_ = .4;
	t_ = 20;
	maxNoEval_ = 200000000;

#ifdef UNIX
	timeval tv;
	gettimeofday(&tv, NULL);
	// :TODO: questo 1. fa cagare come seed fisso
	srand(tv.tv_usec);
#endif
#ifdef WIN32
	srand(GetTickCount());
#endif
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::init()
{
	noParameters_ = parameters_.getNoParameters();
	x_.resize(noParameters_);
	parameters_.getStartingVectorParameters(x_);
	parameters_.setUpperLowerBounds(upperBounds_, lowerBounds_);

	xOpt_.resize(noParameters_);
	v_.resize(noParameters_);
	for (int i = 0; i < noParameters_; ++i)
		v_.at(i) = (upperBounds_.at(i) - lowerBounds_.at(i)) / 2;
	xp_.resize(noParameters_);
	noAccepted_.resize(noParameters_);

	//::cout << "number of parameters (should be 4): " << noParameters_ << std::endl;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setNT(double nt)
{
	nt_ = nt;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setNS(double ns)
{
	ns_ = ns;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setRT(double rt)
{
	rt_ = rt;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setT(double t)
{
	t_ = t;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setMaxNoEval(int maxNoEval)
{
	maxNoEval_ = maxNoEval;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setEpsilon(double epsilon)
{
	epsilon_ = epsilon;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::setNEpsilon(unsigned nEpsilon)
{
	nEpsilon_ = nEpsilon;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::checkBounds(int k) {
	if ((xp_.at(k) < lowerBounds_.at(k)) ||
		(xp_.at(k) > upperBounds_.at(k)))
		xp_.at(k) = lowerBounds_.at(k) + ((upperBounds_.at(k) - lowerBounds_.at(k)) * rand() / RAND_MAX);
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::optimize() {
	objectiveFunction_.setEpsilon(epsilon_);
	objectiveFunction_.setNEpsilon(nEpsilon_);
	objectiveFunction_.init();
	int noEval = 0;
	for (int i = 0; i < noParameters_; ++i)
		noAccepted_.at(i) = 0;
	// DO UNTIL convergence or maxNoEvaluation
	bool terminate = false;
	while ((!terminate) && (noEval < maxNoEval_)) 
	{
		// DO Nt times
		for (int i = 0; i < nt_; ++i) {
			// DO Ns times
			for (int j = 0; j < ns_; ++j) {
				// DO i = 1, n
				for (int k = 0; k < noParameters_; ++k) {
#ifdef LOG_SIMULATED_ANNEALING
					cout << "\ni: " << i << "/" << nt_
						<< " j: " << j << "/" << ns_
						<< " k: " << k << "/" << noParameters_ << endl;
#endif
					//:TODO: check with massimo
					// loro mettono qui (if NEVALS>=MAXEVAL)
					//  return....
					// io penso abbia piu` senso metterlo fuori
					// (condizione while)

#ifdef LOG_SIMULATED_ANNEALING
					cout << "x_ : ";
					for (unsigned int it = 0; it < x_.size(); ++it)
						cout << x_.at(it) << " ";
					cout << endl;
#endif
					// x'_i = x_i + r v_i
					xp_ = x_;

					double factorForV = (2.*rand() / static_cast<double>(RAND_MAX)-1.);
					xp_.at(k) = x_.at(k) + v_.at(k) * factorForV;
					checkBounds(k);

#ifdef LOG_SIMULATED_ANNEALING
					cout << "xp_ : ";
					for (unsigned int it = 0; it < xp_.size(); ++it)
						cout << xp_.at(it) << " ";
					cout << endl;
#endif

					parameters_.setVectorParameters(xp_);

					// objectiveFunction_ ha al suo interno un riferimento
					// a TorqueComputation_ che fa girare il codice del modello
					// per avere come output torque e penalty
					objectiveFunction_.evalFp();
					++noEval;

#ifdef LOG_SIMULATED_ANNEALING
					std::cout << "eval no. " << noEval << std::endl << std::flush;
					objectiveFunction_.printFs();
#endif

					// if f' < f then
					if (objectiveFunction_.isFacceptable()) {
						// X = X'
						x_ = xp_;
						// f = f'
						objectiveFunction_.updateF();  // f_ = fp_;
						// update statistics
						noAccepted_.at(k)++;

#ifdef LOG_SIMULATED_ANNEALING
						cout << "F is ACCEPTABLE\n";
#endif

						// if f' < fopt then
						//if (fp_ < fOpt_) {
						if (objectiveFunction_.isFoptAcceptable()) {
							// Xopt = X'
							xOpt_ = xp_;
							// fOpt = f'
							objectiveFunction_.updateFopt(); // fOpt_ = fp_;

#ifdef LOG_SIMULATED_ANNEALING
							cout << "Fopt is ACCEPTABLE\n";
#endif
						}
					}
					else { // IF f' > f THEN
						double p = objectiveFunction_.computeMetropolisCriteria(t_);
						double randomValue = rand() / static_cast<double>(RAND_MAX);

						if (randomValue < p) {
#ifdef LOG_SIMULATED_ANNEALING
							cout << "through Metropolis " << randomValue << "<" << p;
							cout << " is acceptable\n";
#endif
							x_ = xp_;             // X = X'
							objectiveFunction_.updateF(); // f_ = fp_;
							noAccepted_.at(k)++;
						}
						else {
#ifdef LOG_SIMULATED_ANNEALING
							cout << "through Metropolis NOT ACCEPTABLE\n";
#endif
						}
					}
				} // k
			} // j

			// Adjust V such that half of all trial are accepted

			vector<double> c;
			c.resize(noParameters_);
			for (int h = 0; h < noParameters_; ++h)
				c.at(h) = 2.;

			for (int h = 0; h < noParameters_; ++h) {
				double ratio = noAccepted_.at(h) / ns_;
				if (ratio > 0.6)
					v_.at(h) = v_.at(h) * (1 + c.at(h) * (ratio - 0.6) / 0.4);
				else if (ratio < 0.4)
					v_.at(h) = v_.at(h) / (1 + c.at(h) * ((0.4 - ratio) / 0.4));
				if (v_.at(h) >(upperBounds_.at(h) - lowerBounds_.at(h)))
					v_.at(h) = upperBounds_.at(h) - lowerBounds_.at(h);
			}

			for (int h = 0; h < noParameters_; ++h)
				noAccepted_.at(h) = 0.;
		} // i

		// terminate?
		terminate = objectiveFunction_.terminate();

		// reduce temperature
		t_ = t_ * rt_;
#ifdef LOG_SIMULATED_ANNEALING
		cout << "Temperature reduced at: " << t_ << endl;
#endif
		objectiveFunction_.updateFandFlatest();
		// restart
		x_ = xOpt_;
	}  /* end while */
	//NOTE: the following two lines are important for the static opt
	parameters_.setVectorParameters(xOpt_);
	objectiveFunction_.evalFp();
	//cout << "total evaluations:" << noEval << endl;
	cout << noEval << endl;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
void SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>::getXopt(vector<double>& xOpt) const {
	xOpt = xOpt_;
}

template<typename NMSmodelT, typename Parameters, typename ObjectiveFunction, typename StaticComputationT>
std::ostream& operator<< (std::ostream& output,
	const SimulatedAnnealingBase<NMSmodelT, Parameters, ObjectiveFunction, StaticComputationT>& sa)
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