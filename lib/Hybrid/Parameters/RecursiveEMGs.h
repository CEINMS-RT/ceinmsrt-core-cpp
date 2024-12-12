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

#ifndef RecursiveEMGs_h
#define RecursiveEMGs_h

#include "NMSmodel.h"

#include <vector>
#include <string>

namespace Hybrid {
    namespace Parameters {
        template<typename NMSmodelT>
        class RecursiveEMGs;

        template<typename NMSmodelT>
        std::ostream& operator<< (std::ostream& output, 
                        const RecursiveEMGs<NMSmodelT>& p);


        template<typename NMSmodelT>
        class RecursiveEMGs{

        public:
            RecursiveEMGs(NMSmodelT& subject, 
                                const std::vector< std::string >& muscleNamesWithEMGtoTrack, 
                                const std::vector< std::string >& muscleNamesWithEMGtoPredict);

            int getNoParameters() { return noParameters_; }
            void getStartingVectorParameters(std::vector<double>& x);
            void setVectorParameters(const std::vector<double>& x);
            void setUpperLowerBounds(std::vector<double>& upperBounds, std::vector<double>& lowerBounds);
            friend std::ostream& operator<< <> (std::ostream& output, const RecursiveEMGs& p);
            
        private:
            NMSmodelT& subject_;

            unsigned noParameters_;
            mutable std::vector<double> x_;
            std::vector<double> startingEmgValues_;
            std::vector<unsigned> muscleIndexWithEMGtoTrack_;
            std::vector<unsigned> muscleIndexWithEMGtoPredict_;
            std::vector<unsigned> muscleIndexWithEMGtoOptimize_;
        };
    }
}

#include "RecursiveEMGs.cpp"

#endif