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
#include <string>
using std::string;
#include <stdlib.h>
#include <iostream>

namespace Hybrid {
    namespace Parameters {    
        
        template<typename NMSmodelT>
        RecursiveEMGs<NMSmodelT>::RecursiveEMGs(NMSmodelT& subject, 
                            const vector<string>& muscleNamesWithEMGtoTrack, 
                            const vector<string>& muscleNamesWithEMGtoPredict):
        subject_(subject) {    

            if(!subject_.haveTheseMuscles(muscleNamesWithEMGtoTrack) || !subject_.haveTheseMuscles(muscleNamesWithEMGtoPredict)) {
                
                std::cout << "Error in RecursiveEMGs_Hybrid: muscle name not found\n";
                exit(EXIT_FAILURE);
            }   
            
            subject_.getMusclesIndexFromMusclesList(muscleIndexWithEMGtoTrack_, muscleNamesWithEMGtoTrack); 
            subject_.getMusclesIndexFromMusclesList(muscleIndexWithEMGtoPredict_, muscleNamesWithEMGtoPredict); 

            //concatenate muscleIndexWithEMGtoTrack_ and muscleIndexWithEMGtoPredict_
            muscleIndexWithEMGtoOptimize_.assign(muscleIndexWithEMGtoTrack_.begin(), muscleIndexWithEMGtoTrack_.end());
            muscleIndexWithEMGtoOptimize_.insert( muscleIndexWithEMGtoOptimize_.end(), muscleIndexWithEMGtoPredict_.begin(), muscleIndexWithEMGtoPredict_.end() );

//            muscleIndexWithEMGtoOptimize_.assign(muscleIndexWithEMGtoPredict_.begin(), muscleIndexWithEMGtoPredict_.end());
//            muscleIndexWithEMGtoOptimize_.insert( muscleIndexWithEMGtoOptimize_.end(), muscleIndexWithEMGtoTrack_.begin(), muscleIndexWithEMGtoTrack_.end());
            
            noParameters_ = muscleIndexWithEMGtoOptimize_.size();

            /*for(vector<unsigned>::const_iterator it = muscleIndexWithEMGtoOptimize_.begin(); it != muscleIndexWithEMGtoOptimize_.end(); ++it)
                cout << *it << " ";
            cout << endl;*/
            
        } 


        template<typename NMSmodelT>
        void RecursiveEMGs<NMSmodelT>::getStartingVectorParameters(std::vector<double>& x) {
        
            x.resize(noParameters_);
            
            vector<double> currentEMGs, pastEMGs;
            subject_.getEmgs(currentEMGs);
            subject_.getPastEmgs(pastEMGs);
            
            unsigned indexCt = 0;
            for ( unsigned i = 0; i < muscleIndexWithEMGtoTrack_.size(); ++i, ++indexCt)
                x.at(indexCt) = currentEMGs.at(muscleIndexWithEMGtoTrack_.at(i));

            for ( unsigned i = 0; i < muscleIndexWithEMGtoPredict_.size(); ++i, ++indexCt)
                x.at(indexCt) = pastEMGs.at(muscleIndexWithEMGtoPredict_.at(i));
            
            startingEmgValues_ = currentEMGs;
            for( unsigned i = 0; i < muscleIndexWithEMGtoPredict_.size(); ++i) 
                startingEmgValues_.at(muscleIndexWithEMGtoPredict_.at(i)) = pastEMGs.at(muscleIndexWithEMGtoPredict_.at(i));
               
        }


        template<typename NMSmodelT>
        void RecursiveEMGs<NMSmodelT>::setUpperLowerBounds(vector<double>& upperBounds, vector<double>& lowerBounds) {

            upperBounds.resize(noParameters_); 
            lowerBounds.resize(noParameters_);
            
            for ( unsigned i = 0; i < noParameters_; ++i ) {
                    upperBounds.at(i) = 1.0;
                    lowerBounds.at(i) = 0.0;
            }  
        }


        template<typename NMSmodelT>
        void RecursiveEMGs<NMSmodelT>::setVectorParameters(const vector<double>& x) {
            
            vector<double> emgValues(startingEmgValues_);
            for(unsigned i = 0; i < muscleIndexWithEMGtoOptimize_.size(); ++i) 
                emgValues.at(muscleIndexWithEMGtoOptimize_.at(i)) = x.at(i);
//                 emgValues.at(i) = x.at(i);
            //std::cout << "muscleIndexWithEMGtoOptimize_ size: " << muscleIndexWithEMGtoOptimize_.size() << "\n";
            subject_.setEmgs(emgValues);

           /*cout << "sorted emg (in Parameters)";
            for(unsigned i = 0; i < emgValues.size(); ++i)
                cout << emgValues.at(i) << " ";
            cout << endl;*/
          
        }
            

        template<typename NMSmodelT>
        std::ostream& operator<< (std::ostream& output, const RecursiveEMGs<NMSmodelT>& p) {
            
            output << "Parameters operator<< Not Implemented " << std::endl;
                
            return output;
        }
    
    }
}



