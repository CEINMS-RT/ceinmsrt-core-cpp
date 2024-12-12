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

#include "HybridWeightings.h"
#include "StaticComputation.h"
#include "StaticComputationMode/Default.h"
#include "Parameters/RecursiveEMGs.h"
#include "ObjectiveFunction/MinTorqueTrackEMGs.h"
#include "SimulatedAnnealingBase.h"

#include <iostream>
#include <stdlib.h>

namespace Hybrid {

    
    template <typename NMSmodelT>
    ErrorMinimizerAnnealing<NMSmodelT>::ErrorMinimizerAnnealing(NMSmodelT& subject)
    :subject_(subject) {
        
        subject_.getDoFNames(subjectDofNames_);
        currentDofNames_.resize(subjectDofNames_.size());
        currentExternalTorques_.resize(subjectDofNames_.size());
        weightings_.alpha = 1.;
        weightings_.beta = 0;
        weightings_.gamma = 0;
    }
    
     template<typename NMSmodelT>
    void ErrorMinimizerAnnealing<NMSmodelT>::setMusclesNamesWithEmgToTrack(const vector<string>& musclesNamesWithEmgToTrack) {
        
        musclesNamesWithEmgToTrack_.assign(musclesNamesWithEmgToTrack.begin(), musclesNamesWithEmgToTrack.end());
    }
    

    template<typename NMSmodelT>
    void ErrorMinimizerAnnealing<NMSmodelT>::setMusclesNamesWithEmgToPredict(const vector<string>& musclesNamesWithEmgToPredict) {
        
        musclesNamesWithEmgToPredict_.assign(musclesNamesWithEmgToPredict.begin(), musclesNamesWithEmgToPredict.end());
    }

    
    template<typename NMSmodelT>
    void ErrorMinimizerAnnealing<NMSmodelT>::setSingleExternalTorque(double externalTorque, const string& whichDof) {

        vector<string>::const_iterator it = subjectDofNames_.begin();
        while( *it != whichDof && it != subjectDofNames_.end())
            ++it;
        if(*it == whichDof) {
            unsigned pos = std::distance<vector<string>::const_iterator>(subjectDofNames_.begin(), it);
            currentExternalTorques_.at(pos) = externalTorque;
            currentDofNames_.at(pos) = whichDof;
        }
        else {
            std::cout << "ErrorMinimizer::setSingleExternalTorque ERROR\n" << whichDof << " not found in the subject\n";            
            exit(EXIT_FAILURE);
        }
            
    }

    
    template<typename NMSmodelT>
    void ErrorMinimizerAnnealing<NMSmodelT>::setAllExternalTorques(vector<double> externalTorques, const vector<string>& dofs) {
        
        currentExternalTorques_ = externalTorques;
        currentDofNames_ = dofs;
    }
    
    template <typename NMSmodelT>
    void ErrorMinimizerAnnealing<NMSmodelT>::minimize() {
      
        if (currentDofNames_ != subjectDofNames_) {
            std::cout << "ERROR: the joints names from the external torques are different from the subject ones\n";
            exit(EXIT_FAILURE);
        } // commented out so the DOFs for optimization can be chosen by the size of the external torques
            
        typedef StaticComputation<NMSmodelT, StaticComputationMode::Default<NMSmodelT> > MyStaticComputation;
        typedef SimulatedAnnealingBase<
                                       NMSmodelT, 
                                       Parameters::RecursiveEMGs<NMSmodelT>,
                                       ObjectiveFunction::MinTorqueTrackEMGs<MyStaticComputation>,
                                       MyStaticComputation
                                      > MySiman;
          
        //std::cout << "Optimizing timeframe " << currentTime_ << std::endl;
         MyStaticComputation staticComputation(subject_, 
                                              musclesNamesWithEmgToTrack_, 
                                              musclesNamesWithEmgToPredict_);
	      staticComputation.setExternalTorques(currentExternalTorques_,
                                             currentDofNames_);
        staticComputation.setTime(currentTime_);
        
        MySiman siman(subject_, 
                      musclesNamesWithEmgToTrack_, 
                      musclesNamesWithEmgToPredict_, 
                      weightings_, 
                      staticComputation,
                      performanceCriterion_);

		siman.setMaxNoEval(maxNoEval_);
		siman.setNS(ns_);
		siman.setNT(nt_);
		siman.setRT(rt_);
		siman.setT(t_);
		siman.setEpsilon(epsilon_);
		siman.setNEpsilon(nEpsilon_);
		siman.init();
        //std::cout << siman;
        siman.optimize();
        
        /*std::vector<std::string> muscleNames;
        subject_.getMuscleNames(muscleNames);
        int noMuscles = subject_.getNoMuscles();
        std::vector <std::string> dofNames;
        subject_.getDoFNames(dofNames);
        int noDofs = subject_.getNoDoF();

        std::cout << "Number of muscles: " << noMuscles << std::endl;
        for (int i = 0; i < noMuscles; i++)
            std::cout << muscleNames.at(i);
        std::cout << "Number of dofs: " << noDofs << std::endl;
        for (int i = 0; i < noDofs; i++)
            std::cout << dofNames.at(i);*/

        vector<double> xOpt;
        siman.getXopt(xOpt);
        /*cout << "final xOpt: ";
        for(unsigned i = 0; i < xOpt.size(); ++i)
            cout << xOpt.at(i) << " ";
        cout << endl;*/
        }

};