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

#include <vector>
using std::vector;
#include <string>
using std::string;
#include <iostream>
using std::cout;

#include <stdlib.h>

namespace Hybrid {
   
    //TODO:: ricordarsi di far iniziare i muscoli da predirre con il valore al tempo t1
    template<typename NMSmodelT, typename StaticComputationMode>
    StaticComputation<NMSmodelT, StaticComputationMode>::StaticComputation(NMSmodelT& subject, 
                                                                           const vector<string>& muscleNamesWithEMGtoTrack, 
                                                                           const vector<string>& muscleNamesWithEMGtoPredict): 
    subject_(subject), computationMode_(subject) {
        
       if(!subject_.haveTheseMuscles(muscleNamesWithEMGtoTrack) || !subject_.haveTheseMuscles(muscleNamesWithEMGtoPredict)) {
            std::cout << "Error in RecursiveEMGs_Hybrid: muscle name not found\n";
            exit(EXIT_FAILURE);
        }   
        
        subject_.getMusclesIndexFromMusclesList(muscleIndexWithEMGtoTrack_, muscleNamesWithEMGtoTrack); 
        subject_.getMusclesIndexFromMusclesList(muscleIndexWithEMGtoPredict_, muscleNamesWithEMGtoPredict); 

        //concatenate muscleIndexWithEMGtoTrack_ and muscleIndexWithEMGtoPredict_
        muscleIndexWithEMGtoOptimize_.assign(muscleIndexWithEMGtoTrack_.begin(), muscleIndexWithEMGtoTrack_.end());
        muscleIndexWithEMGtoOptimize_.insert( muscleIndexWithEMGtoOptimize_.end(), muscleIndexWithEMGtoPredict_.begin(), muscleIndexWithEMGtoPredict_.end() );    
    
        vector<double> currentEMGData;
        subject_.getEmgs(currentEMGData);
        
        for(unsigned i = 0; i < muscleIndexWithEMGtoTrack_.size(); ++i)
            initialValueOfTrackedEMGs_.push_back(currentEMGData.at(muscleIndexWithEMGtoTrack_.at(i)));
    }


    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::setExternalTorques(const vector<double>& externalTorques, const vector<string>& dofNames) {
        
        vector<string> dofNamesFromSubject;
        subject_.getDoFNames(dofNamesFromSubject);
        if(dofNamesFromSubject != dofNames)
        {
            cout << "StaticComputation: dofNames from the subject and external torques dof names are different\n";
            exit(EXIT_FAILURE);
        }
        
        externalTorques_ = externalTorques;
    }


    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getExternalTorques(vector<double>& externalTorques) const {
        
        externalTorques = externalTorques_;
    }


    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getTorques(vector<double>& torques) {
        
        computationMode_.getTorques(torques);
    }


    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getCurrentEMGs(vector<double>& emgValues) const {
        
        subject_.getEmgs(emgValues);
    }

    
    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getInitialValuesOfTrackedEMGs(vector<double>& emgValues) const {
        
        emgValues.assign(initialValueOfTrackedEMGs_.begin(), initialValueOfTrackedEMGs_.end());
    }
    
    
     
    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getAdjustedValuesOfTrackedEMGs(vector<double>& emgValues) const {
        
        emgValues.resize(muscleIndexWithEMGtoTrack_.size());
        vector<double> currentEMGData;
        subject_.getEmgs(currentEMGData);
        
        for(unsigned i = 0; i < muscleIndexWithEMGtoTrack_.size(); ++i)
            emgValues.at(i) = currentEMGData.at(muscleIndexWithEMGtoTrack_.at(i));
        
    }
    
    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getCurrentMuscleForcesRelative(vector<double>& muscleForcesRel) const {
        vector<double> muscleForces;
        subject_.getMuscleForces(muscleForces);
        /*std::cout << "muscle forces:" << std::endl;
        for (unsigned i = 0; i < muscleForces.size(); ++i)
        {
            std::cout << muscleForces.at(i) << std::endl;
        }*/

        vector<MuscleParameters> parameters;
        subject_.getMusclesParameters(parameters);
        /*std::cout << "max isometric forces:" << std::endl;
        for (unsigned i = 0; i < parameters.size(); ++i)
        {
            std::cout << parameters.at(i).getMaxIsometricForce() << std::endl;
        }*/
        muscleForcesRel.resize(muscleForces.size());
        //std::cout << "relative muscle forces:" << std::endl;
        for (unsigned i = 0; i < muscleForces.size(); ++i)
        {
            muscleForcesRel.at(i) = muscleForces.at(i) / parameters.at(i).getMaxIsometricForce();
            //std::cout << muscleForcesRel.at(i) << std::endl;
        }
    }

    template<typename NMSmodelT, typename StaticComputationMode>
    void StaticComputation<NMSmodelT, StaticComputationMode>::getCurrentMuscleEnergy(vector<double>& muscleEnergy) const {
        double K = 400000.; // ??

        //subject_.updateState_HYBRID();

        vector<double> muscleForces;
        subject_.getMuscleForces(muscleForces);

        //std::cout << "muscleForces length: " << muscleForces.size() << std::endl;

        vector<MuscleParameters> parameters;
        subject_.getMusclesParameters(parameters);

        //std::cout << "parameters length: " << parameters.size() << std::endl;

        vector<double> activations;
        subject_.getActivations(activations);

        //std::cout << "activations length: " << activations.size() << std::endl;

        vector<double> fibreLengths;
        subject_.getFiberLengths(fibreLengths);

        //std::cout << "fibreLengths length: " << fibreLengths.size() << std::endl;
        
        muscleEnergy.resize(muscleForces.size());

        /*for (unsigned i = 0; i < muscleForces.size(); ++i)
        {
            muscleEnergy.at(i) = fibreLengths.at(i) * muscleForces.at(i) + activations.at(i) * fibreLengths.at(i) * parameters.at(i).getMaxIsometricForce() / K;
            std::cout << fibreLengths.at(i) * muscleForces.at(i) << "\t" << activations.at(i) * fibreLengths.at(i) * parameters.at(i).getMaxIsometricForce() / K << std::endl;
        }*/

        std::vector<std::string> muscleNames;
        subject_.getMuscleNames(muscleNames);

        double c1 = 1.;
        double c2 = 10000000.;
        double sigma = 1000000.;
        double PCSA;
        
        for (unsigned i = 0; i < muscleForces.size(); ++i)
        {
            PCSA = parameters.at(i).getMaxIsometricForce() / K;
            muscleEnergy.at(i) = fibreLengths.at(i) * (c1*muscleForces.at(i) + c2* PCSA*(muscleForces.at(i)/(sigma*PCSA)) * (muscleForces.at(i)/(sigma* PCSA)) );
            //std::cout << c1 * muscleForces.at(i) << "\t" << c2 * PCSA * (muscleForces.at(i)/(sigma*PCSA)) * (muscleForces.at(i) / (sigma*PCSA))  << std::endl;
            /*std::cout << "muscle name: " << muscleNames.at(i) << std::endl;
            std::cout << "fibreLengths: " << fibreLengths.at(i) << std::endl;
            std::cout << "muscleForces: " << muscleForces.at(i) << std::endl;
            std::cout << "PCSA: " << PCSA << std::endl;
            std::cout << "muscleEnergy: " << muscleEnergy.at(i) << std::endl;*/
        }
    }
}


















