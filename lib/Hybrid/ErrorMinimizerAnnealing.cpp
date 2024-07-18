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