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
#include <vector>
using std::vector;
#include "TrialData.h"

template<typename NMSmodelT>
ComputationMode_Default<NMSmodelT>::ComputationMode_Default(NMSmodelT& subject):
subject_(subject) { }


template<typename NMSmodelT>
void ComputationMode_Default<NMSmodelT>::setTrials( const vector<TrialData>& trials)
{

    trials_ = trials;
    unsigned nMuscles = subject_.getNoMuscles();
    parametersT1_.assign(nMuscles, MuscleParameters());
    musclesToUpdate_.assign(nMuscles, true);
    
    
    // resizing forceDataT1_
    // forceDataT1_ forces at previous calibration step
    forceDataT1_.resize(trials_.size());
    for(unsigned int i = 0; i < trials_.size(); ++i) {
        forceDataT1_.at(i).resize(trials_.at(i).noLmtSteps_);
        for(unsigned int j = 0; j < trials_.at(i).noLmtSteps_; ++j)
            forceDataT1_.at(i).at(j).resize(nMuscles);
    }
    
    // resizing emgDataT1_
    // forceDataT1_ forces at previous calibration step
    activationDataT1_.resize(trials_.size());
    for(unsigned int i = 0; i < trials_.size(); ++i) {
        activationDataT1_.at(i).resize(trials_.at(i).noEmgSteps_);
        for(unsigned int j = 0; j < trials_.at(i).noEmgSteps_; ++j)
            activationDataT1_.at(i).at(j).resize(nMuscles);            
    }
    
}


template<typename NMSmodelT>
void ComputationMode_Default<NMSmodelT>::getMusclesToUpdate() {

    vector<MuscleParameters> currentParameters;
    musclesToUpdate_.clear();
    subject_.getMusclesParameters(currentParameters);
    for(unsigned int i = 0; i < currentParameters.size(); ++i)
        if(!(currentParameters.at(i) == parametersT1_.at(i)))
            musclesToUpdate_.push_back(i);
    parametersT1_ = currentParameters;
}


template<typename NMSmodelT>
void ComputationMode_Default<NMSmodelT>::initFiberLengthTraceCurves(unsigned trialIndex) {
    
    unsigned const ct = trialIndex;
    unsigned k = 0; // k is the index for lmt and ma data
    double lmtTime = trials_.at(ct).lmtTimeSteps_.at(k);
    double emgTime = trials_.at(ct).emgTimeSteps_.at(0);

    subject_.resetFibreLengthTraces(musclesToUpdate_);
    
    for (unsigned i = 0; i < trials_.at(ct).noEmgSteps_; ++i) {
    
        // set emg to model and save the activations
        subject_.setActivations(activationDataT1_.at(ct).at(i));
        emgTime = trials_.at(ct).emgTimeSteps_.at(i);
        subject_.setTime_emgs_updateActivations_pushState_selective(emgTime, trials_.at(ct).emgData_.at(i), musclesToUpdate_);
        vector<double> currentActivations;
        subject_.getActivations(currentActivations);
        for (int mi = 0; mi < subject_.getNoMuscles(); ++mi)
            activationDataT1_.at(ct).at(i).at(mi) = currentActivations.at(mi);
        
        // set lmt 
        if ( (lmtTime <= emgTime) && (k < trials_.at(ct).noLmtSteps_)) {
            subject_.setMuscleTendonLengthsSelective(trials_.at(ct).lmtData_.at(k), musclesToUpdate_);
            subject_.updateFibreLengths_OFFLINEPREP(musclesToUpdate_);
            ++k;
            if (k < trials_.at(ct).noLmtSteps_)
                lmtTime = trials_.at(ct).lmtTimeSteps_.at(k);  
        }
    }
    subject_.updateFibreLengthTraces(musclesToUpdate_); 
}


template<typename NMSmodelT>
void ComputationMode_Default<NMSmodelT>::computeTorques(vector< vector< std::vector<double> > >& torques) {

	getMusclesToUpdate();
	double lmtTimePast = -1;
	std::vector<std::string> muscleName, dofName;
	subject_.getMuscleNames(muscleName);
	subject_.getDoFNames(dofName);
    // i is for the number of trials
	for (unsigned int ct = 0; ct < trials_.size(); ++ct)
	{

		// initializing fiber-length curves for the trial ct
		initFiberLengthTraceCurves(ct);

		int k = 0; // k is the index for lmt and ma data
		unsigned kEMG = 0; // kEMG is the index for EMG data when noEmgSteps_<noLmtSteps_

		if (trials_.at(ct).noEmgSteps_ >= trials_.at(ct).noLmtSteps_) //if more emg sample than angle
		{

			double lmtTime = trials_.at(ct).lmtTimeSteps_.at(k);

			// Let's start going through the EMG, lmt, and ma data...  
			for (int i = 0; i < trials_.at(ct).noEmgSteps_; ++i)
			{

				double emgTime = trials_.at(ct).emgTimeSteps_.at(i);
				//subject_.setActivations(activationDataT1_.at(ct).at(i));
				if (emgTime < lmtTime)
					subject_.setTime_emgs_updateActivations_pushState_selective(emgTime, trials_.at(ct).emgData_.at(i), musclesToUpdate_);

				if ((lmtTime <= emgTime) && (k < trials_.at(ct).noLmtSteps_))
				{
					subject_.setTime(emgTime);
					subject_.setEmgsSelective(trials_.at(ct).emgData_.at(i), musclesToUpdate_);
					subject_.setMuscleForces(forceDataT1_.at(ct).at(k));
					subject_.setMuscleTendonLengthsSelective(trials_.at(ct).lmtData_.at(k), musclesToUpdate_);
					for (int j = 0; j < trials_.at(ct).noDoF_; ++j)
						subject_.setMomentArms(trials_.at(ct).maData_.at(j).at(k), j);
					subject_.updateState(musclesToUpdate_); // subject_.updateState_OFFLINE(musclesToUpdate_); removed due to online test
					subject_.pushState(musclesToUpdate_);
					vector<double> currentTorques, currentForces;
					subject_.getTorques(currentTorques);
					subject_.getMuscleForces(currentForces);
					// when I'm done with the moment arm, I can ask for the new torque, and put it in the matrix
					for (int j = 0; j < trials_.at(ct).noDoF_; ++j)
						torques.at(ct).at(j).at(k) = currentTorques.at(j);
					for (int j = 0; j < subject_.getNoMuscles(); ++j)
						forceDataT1_.at(ct).at(k).at(j) = currentForces.at(j);

					++k;
					if (k < trials_.at(ct).noLmtSteps_)
						lmtTime = trials_.at(ct).lmtTimeSteps_.at(k);

#ifdef DEBUG
					cout << endl << endl << "EmgTime: " << emgTime << endl << "EMG" << endl;

					for(unsigned int l=0; l < trials_.at(ct).emgData_.at(i).size(); ++l)
						cout << trials_.at(ct).emgData_.at(i).at(l) << "\t" ;
					cout << endl << "LmtTime: " << lmtTime << endl;

					for(unsigned int l=0; l < trials_.at(ct).lmtData_.at(k).size(); ++l)
						cout << trials_.at(ct).lmtData_.at(k).at(l) << "\t";
					cout << endl << "MomentArms" << endl;

					for(unsigned int l=0; l < trials_.at(ct).maData_.at(0).at(k).size(); ++l)
						cout << trials_.at(ct).maData_.at(0).at(k).at(l) << "\t";
					cout << "\ncurrent torque: " << currentTorques.at(0);

					cout << endl << "----------------------------------------" << endl;
#endif
				}
				vector<double> currentActivations;
				subject_.getActivations(currentActivations);
				for (int mi = 0; mi < subject_.getNoMuscles(); ++mi)
					activationDataT1_.at(ct).at(i).at(mi) = currentActivations.at(mi);
			}
		}
		else // if more angle than emg sample (ex: exoskeleton)
		{
			double emgTime = trials_.at(ct).emgTimeSteps_.at(kEMG);

			for (int i = 0; i < trials_.at(ct).noLmtSteps_; ++i)
			{
				double  lmtTime = trials_.at(ct).lmtTimeSteps_.at(i);

				vector<double> currentTorques, currentForces;
				if (lmtTimePast != lmtTime)
				{
					lmtTimePast = lmtTime;
					if ((emgTime < lmtTime) && (kEMG < trials_.at(ct).noEmgSteps_))
					{
						subject_.setTime_emgs_updateActivations_pushState_selective(emgTime, trials_.at(ct).emgData_.at(kEMG), musclesToUpdate_);

					/*	for (unsigned int l = 0; l < trials_.at(ct).emgData_.at(k).size(); ++l)
						cout << trials_.at(ct).emgData_.at(k).at(l) << "\t";
						cout << endl << i << " EMGTime: " << emgTime << endl;*/
						++kEMG;
						if (kEMG < trials_.at(ct).noEmgSteps_)
							emgTime = trials_.at(ct).emgTimeSteps_.at(kEMG);
					}
					if ((lmtTime <= emgTime) && (kEMG < trials_.at(ct).noEmgSteps_))
					{
						subject_.setMuscleForces(forceDataT1_.at(ct).at(i));
						subject_.setTime(lmtTime);
						subject_.setEmgsSelective(trials_.at(ct).emgData_.at(kEMG), musclesToUpdate_);
						subject_.setMuscleTendonLengths(trials_.at(ct).lmtData_.at(i));
						for (int j = 0; j < trials_.at(ct).noDoF_; ++j)
							subject_.setMomentArms(trials_.at(ct).maData_.at(j).at(i), j);

						subject_.updateState(musclesToUpdate_);
						subject_.pushState(musclesToUpdate_);
						subject_.getTorques(currentTorques);
						subject_.getMuscleForces(currentForces);
						for (int j = 0; j < trials_.at(ct).noDoF_; ++j)
						{
							torques.at(ct).at(j).at(i) = currentTorques.at(j);
						}
						for (int j = 0; j < subject_.getNoMuscles(); ++j)
							forceDataT1_.at(ct).at(i).at(j) = currentForces.at(j);

						std::vector<double> activation, fiberLengths, fiberVelocities;
						subject_.getActivations(activation);
						subject_.getFiberLengths(fiberLengths);
						subject_.getFiberVelocities(fiberVelocities);

						/*for (unsigned int l = 0; l < muscleName.size(); ++l)
							cout << muscleName.at(l) << "\t";
						cout << endl << endl << "EmgTime: " << emgTime << endl << "EMG" << endl;
						for (unsigned int l = 0; l < trials_.at(ct).emgData_.at(kEMG).size(); ++l)
							cout << trials_.at(ct).emgData_.at(kEMG).at(l) << "\t";
						cout << endl << "getActivations: " << endl;
						for (unsigned int l = 0; l < activation.size(); ++l)
							cout << activation.at(l) << "\t";
						cout << endl << "getFiberLengths: " << endl;
						for (unsigned int l = 0; l < fiberLengths.size(); ++l)
							cout << fiberLengths.at(l) << "\t";
						cout << endl << "getFiberVelocities: " << endl;
						for (unsigned int l = 0; l < fiberVelocities.size(); ++l)
							cout << fiberVelocities.at(l) << "\t";
						cout << endl << "getMuscleForces: " << endl;
						for (unsigned int l = 0; l < currentForces.size(); ++l)
							cout << currentForces.at(l) << "\t";
						cout << endl;
						for (unsigned int l = 0; l < dofName.size(); ++l)
							cout << dofName.at(l) << "\t";
						cout << endl << "currentTorques: " << lmtTime << endl;
						for (unsigned int l = 0; l < currentTorques.size(); ++l)
							cout << currentTorques.at(l) << "\t";
						cout << endl;*/

						vector<double> currentActivations;
						subject_.getActivations(currentActivations);
						for (int mi = 0; mi < subject_.getNoMuscles(); ++mi)
							activationDataT1_.at(ct).at(kEMG).at(mi) = currentActivations.at(mi);

					}
				}
			}
		}
	}
	//exit(-1);
}


template<typename NMSmodelT>
ComputationMode_Default<NMSmodelT>& ComputationMode_Default<NMSmodelT>::operator=(const ComputationMode_Default<NMSmodelT>& orig) {
    
    subject_         = orig.subject_;
    trials_          = orig.trials_;
    parametersT1_     = orig.parametersT1_;
    forceDataT1_      = orig.forceDataT1_;
    activationDataT1_ = orig.activationDataT1_;
    musclesToUpdate_  = orig.musclesToUpdate_;
    return *this;
}
