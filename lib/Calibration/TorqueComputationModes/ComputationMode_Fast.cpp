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

#include <vector>
using std::vector;
#include "TrialData.h"

#include "ComputationMode_Fast.h"

template<typename NMSmodelT>
ComputationMode_Fast<NMSmodelT>::ComputationMode_Fast(NMSmodelT& subject):
subject_(subject) { }


template<typename NMSmodelT>
void ComputationMode_Fast<NMSmodelT>::setTrials( const vector<TrialData>& trials) {

    trials_ = trials;
    unsigned nMuscles = subject_.getNoMuscles();
    parametersT1_.assign(nMuscles, MuscleParameters());
    musclesToUpdate_.assign(nMuscles, true);
    
    
    // resizing forceDataT1_
    // forceDataT1_ forces at previous calibration step
    forceDataT1_.resize(trials_.size());
    for(unsigned int i = 0; i < trials_.size(); ++i) {
        forceDataT1_.at(i).resize(trials_.at(i).noLmtSteps_);
        for(unsigned int j = 0; j < (unsigned) trials_.at(i).noLmtSteps_; ++j)
            forceDataT1_.at(i).at(j).resize(nMuscles);
    }
     
    normFiberVelDataT1_.resize(trials_.size());
    for(unsigned int i = 0; i < trials_.size(); ++i) {
        normFiberVelDataT1_.at(i).resize(trials_.at(i).noLmtSteps_);
        for(unsigned int j = 0; j < (unsigned) trials_.at(i).noLmtSteps_; ++j)
            normFiberVelDataT1_.at(i).at(j).resize(nMuscles);
    }
}


template<typename NMSmodelT>
void ComputationMode_Fast<NMSmodelT>::getMusclesToUpdate() {

    vector<MuscleParameters> currentParameters;
    musclesToUpdate_.clear();
    subject_.getMusclesParameters(currentParameters);
    for(unsigned int i = 0; i < currentParameters.size(); ++i)
        if(!(currentParameters.at(i) == parametersT1_.at(i)))
            musclesToUpdate_.push_back(i);
    parametersT1_ = currentParameters;
}


template<typename NMSmodelT>
void ComputationMode_Fast<NMSmodelT>::initFiberLengthTraceCurves(unsigned trialIndex) {
    
    unsigned const ct = trialIndex;
    subject_.resetFibreLengthTraces(musclesToUpdate_);
    
    for (unsigned j = 0; j <  trials_.at(ct).lmtTimeSteps_.size(); ++j) {
        double lmtTime = trials_.at(ct).lmtTimeSteps_.at(j);
        //subject_.setTime(lmtTime);
        subject_.setMuscleTendonLengthsSelective(trials_.at(ct).lmtData_.at(j), musclesToUpdate_);
        //subject_.updateFibreLengths_OFFLINEPREP(musclesToUpdate_);
    }
    //subject_.updateFibreLengthTraces(musclesToUpdate_); 
}

template<typename NMSmodelT>
void ComputationMode_Fast<NMSmodelT>::computeTorques(vector< vector< std::vector<double> > >& torques, std::vector< std::vector< double > >& penalties) {

    getMusclesToUpdate();
	double lmtTimePast = -1;
	/*
	for (int i = 0; i < trials_.at(0).noEmgSteps_; ++i)
	{

		std::cout << trials_.at(0).emgTimeSteps_.at(i) << '\t';
	}
	std::cout << endl;

	for (int i = 0; i < trials_.at(0).noLmtSteps_; ++i)
	{

		std::cout << trials_.at(0).lmtTimeSteps_.at(i) << '\t';
	}
	std::cout << endl;
	exit(-1);*/

    // i is for the number of trials
    for (unsigned int ct = 0 ; ct < trials_.size(); ++ct)
	{
        initFiberLengthTraceCurves(ct);
        unsigned k = 0; // k is the index for lmt and ma data

		if (trials_.at(ct).noEmgSteps_>= trials_.at(ct).noLmtSteps_) //if more emg sample than angle
		{
			double lmtTime = trials_.at(ct).lmtTimeSteps_.at(k);

			for (int i = 0; i < trials_.at(ct).noEmgSteps_; ++i)
			{
				
				double emgTime = trials_.at(ct).emgTimeSteps_.at(i);
				if (emgTime < lmtTime)
					subject_.setTime_emgs_updateActivations_pushState_selective(emgTime, trials_.at(ct).emgData_.at(i), musclesToUpdate_);

				if ((lmtTime <= emgTime) && (k < (unsigned) trials_.at(ct).noLmtSteps_))
				{
					subject_.setMuscleForces(forceDataT1_.at(ct).at(k));
					subject_.setEmgs(trials_.at(ct).emgData_.at(i)); // Update EMG for the model
					subject_.updateActivations(); // conpute activation
					subject_.pushState();
					subject_.setTime(lmtTime);
					//subject_.setEmgsSelective(trials_.at(ct).emgData_.at(i), musclesToUpdate_);
					subject_.setMuscleTendonLengths(trials_.at(ct).lmtData_.at(k));
					for (int j = 0; j < trials_.at(ct).noDoF_; ++j)
						subject_.setMomentArms(trials_.at(ct).maData_.at(j).at(k), j);

					subject_.updateState(musclesToUpdate_);
					subject_.pushState(musclesToUpdate_);
					vector<double> currentTorques, currentForces, fibvel;
					double currentPenalty;
					subject_.getTorques(currentTorques);
					subject_.getMuscleForces(currentForces);
					subject_.getFiberVelocities(fibvel);
					currentPenalty = subject_.getMusclePenalty(musclesToUpdate_);

					for (int j = 0; j < trials_.at(ct).noDoF_; ++j) {
						torques.at(ct).at(j).at(k) = currentTorques.at(j);
						penalties.at(ct).at(k) = currentPenalty;
					}
					for (int j = 0; j < subject_.getNoMuscles(); ++j)
						forceDataT1_.at(ct).at(k).at(j) = currentForces.at(j);

					++k;
					if (k < (unsigned) trials_.at(ct).noLmtSteps_)
						lmtTime = trials_.at(ct).lmtTimeSteps_.at(k);

					
				//	cout << lmtTime;

				/*	for (unsigned int l = 0; l < fibvel.size(); ++l)
						cout << "\t" << fibvel.at(l);

				cout << endl;*/

				/*for (unsigned int l = 0; l < currentForces.size(); ++l)
					cout << "\t" << currentForces.at(l);

				cout << endl;*/

					/*for (unsigned int l = 0; l < trials_.at(ct).lmtData_.at(k).size(); ++l)
						cout << "\t" << trials_.at(ct).lmtData_.at(k).at(l);

				cout << endl;*/

					/*cout << endl << "LmtTime: " << lmtTime << endl;

					for (unsigned int l = 0; l < trials_.at(ct).lmtData_.at(k).size(); ++l)
						cout << trials_.at(ct).lmtData_.at(k).at(l) << "\t";
					cout << endl << "MomentArms" << endl;

					for (unsigned int l = 0; l < trials_.at(ct).maData_.at(0).at(k).size(); ++l)
						cout << trials_.at(ct).maData_.at(0).at(k).at(l) << "\t";
					for (int i = 0; i < currentTorques.size(); i++)
						cout << '\t' << currentTorques.at(i);
					cout << endl;
					
					for (unsigned int l = 0; l < forceDataT1_.at(ct).at(k).size(); ++l)
						cout << forceDataT1_.at(ct).at(k).at(l) << "\t";
					cout << endl << "Forces: " << lmtTime << endl;

					for (unsigned int l = 0; l < trials_.at(ct).emgData_.at(i).size(); ++l)
						cout << trials_.at(ct).emgData_.at(i).at(l) << "\t";
					cout << endl << "EMG: " << lmtTime << endl;

					if (isnan(currentTorques.at(0)))
						exit(-1);

					cout << endl << "----------------------------------------" << endl;*/

				}
			}
		}
		else // if more angle than emg sample (ex: exoskeleton)
		{
			double emgTime = trials_.at(ct).emgTimeSteps_.at(k);

			for (int i = 0; i < trials_.at(ct).noLmtSteps_; ++i)
			{
				double  lmtTime = trials_.at(ct).lmtTimeSteps_.at(i);
				if (lmtTimePast != lmtTime)
				{
					lmtTimePast = lmtTime;
					if ((emgTime < lmtTime) && (k < (unsigned) trials_.at(ct).noEmgSteps_))
					{
						//subject_.setTime_emgs_updateActivations_pushState_selective(emgTime, trials_.at(ct).emgData_.at(k), musclesToUpdate_);
						
						/*for (unsigned int l = 0; l < trials_.at(ct).emgData_.at(k).size(); ++l)
							cout << trials_.at(ct).emgData_.at(k).at(l) << "\t";
						cout << endl << i << " EMGTime: " << emgTime << endl;*/
						++k;
						if (k < (unsigned) trials_.at(ct).noEmgSteps_)
							emgTime = trials_.at(ct).emgTimeSteps_.at(k);
					}
					if ((lmtTime <= emgTime) && (k < (unsigned) trials_.at(ct).noEmgSteps_))
					{
						subject_.setMuscleForces(forceDataT1_.at(ct).at(i));
						subject_.setTime(lmtTime);
						subject_.setEmgsSelective(trials_.at(ct).emgData_.at(k), musclesToUpdate_);
						subject_.setMuscleTendonLengths(trials_.at(ct).lmtData_.at(i));
						for (int j = 0; j < trials_.at(ct).noDoF_; ++j)
							subject_.setMomentArms(trials_.at(ct).maData_.at(j).at(i), j);

						subject_.updateState(musclesToUpdate_);
						subject_.pushState(musclesToUpdate_);
						vector<double> currentTorques, currentForces;
						double currentPenalty;
						subject_.getTorques(currentTorques);
						subject_.getMuscleForces(currentForces);
						currentPenalty = subject_.getMusclePenalty(musclesToUpdate_);
						for (int j = 0; j < trials_.at(ct).noDoF_; ++j) {
							torques.at(ct).at(j).at(i) = currentTorques.at(j);
							penalties.at(ct).at(i) = currentPenalty;
						}
						for (int j = 0; j < subject_.getNoMuscles(); ++j)
							forceDataT1_.at(ct).at(i).at(j) = currentForces.at(j);

						std::vector<double> activation, fiberLengths, fiberVelocities;
						subject_.getActivations(activation);
						subject_.getFiberLengths(fiberLengths);
						subject_.getFiberVelocities(fiberVelocities);

						//++k;
						//if (k < trials_.at(ct).noEmgSteps_)
						//emgTime = trials_.at(ct).emgTimeSteps_.at(k);

						/*if (currentTorques.at(1) == 0 || isnan(currentTorques.at(1)))
						{
						cout << endl << endl << k << " EmgTime: " << emgTime << endl << "EMG" << endl;

						for (unsigned int l = 0; l < trials_.at(ct).emgData_.at(k).size(); ++l)
							cout << trials_.at(ct).emgData_.at(k).at(l) << "\t";
						cout << endl << i << " LmtTime: " << lmtTime << endl;

						for (unsigned int l = 0; l < trials_.at(ct).lmtData_.at(i).size(); ++l)
							cout << trials_.at(ct).lmtData_.at(i).at(l) << "\t";

						cout << "\ncurrent torque: " << currentTorques.at(1) << endl;

						for (unsigned int l = 0; l < forceDataT1_.at(ct).at(i).size(); ++l)
							cout << forceDataT1_.at(ct).at(i).at(l) << "\t";
						cout << endl << "Forces: " << lmtTime << endl;

						cout << endl;
						for (unsigned int l = 0; l < activation.size(); ++l)
							cout << activation.at(l) << "\t";
						cout << endl << "activation: " << lmtTime << endl;
						cout << endl;
						for (unsigned int l = 0; l < fiberLengths.size(); ++l)
							cout << fiberLengths.at(l) << "\t";
						cout << endl << "fiberLengths: " << lmtTime << endl;
						cout << endl;
						for (unsigned int l = 0; l < fiberVelocities.size(); ++l)
							cout << fiberVelocities.at(l) << "\t";
						cout << endl << "fiberVelocities: " << lmtTime << endl;

						cout << endl << "LMT" << endl;
						for (unsigned int l = 0; l < trials_.at(ct).lmtData_.at(i).size(); ++l)
							cout << trials_.at(ct).lmtData_.at(i).at(l) << "\t";
						cout << endl << "MomentArms" << endl;

						for (unsigned int v = 0; v < trials_.at(ct).maData_.size(); ++v)
						{
							for (unsigned int l = 0; l < trials_.at(ct).maData_.at(v).at(i).size(); ++l)
								cout << trials_.at(ct).maData_.at(v).at(i).at(l) << "\t";
							cout << endl;
						}

						cout << endl << "----------------------------------------" << endl;
						}

						/*if (isnan(currentTorques.at(1)))
						{


						exit(-1);
						}*/
					}
				}
			}
		}      
	}
	//exit(-1);
}


template<typename NMSmodelT>
void ComputationMode_Fast<NMSmodelT>::computePenalties(std::vector< std::vector<double> >& penalties) {
 
    
    
}


template<typename NMSmodelT>
ComputationMode_Fast<NMSmodelT>& ComputationMode_Fast<NMSmodelT>::operator=(const ComputationMode_Fast<NMSmodelT>& orig) {
    
    subject_          = orig.subject_;
    trials_           = orig.trials_;
    parametersT1_     = orig.parametersT1_;
    forceDataT1_      = orig.forceDataT1_;
    normFiberVelDataT1_ = orig.normFiberVelDataT1_;
    musclesToUpdate_  = orig.musclesToUpdate_;
    return *this;
}

