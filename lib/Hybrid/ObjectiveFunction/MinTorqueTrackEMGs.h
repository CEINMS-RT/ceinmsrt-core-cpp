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

#ifndef MinTorqueTrackEMGs_h
#define MinTorqueTrackEMGs_h

#include "HybridWeightings.h"

#include <vector>



namespace Hybrid {
    namespace ObjectiveFunction {
        
        template<typename StaticComputationT>
        class MinTorqueTrackEMGs {

        public:
            MinTorqueTrackEMGs( StaticComputationT& staticComputation,
                                    double epsilon, double nEpsilon, HybridWeightings hybridParameters, std::string performanceCriterion);
        
            void   evalFp();
            bool   isFacceptable();
            void   updateF();
            bool   isFoptAcceptable();
            void   updateFopt();
            bool   terminate();
            void   updateFandFlatest();
            void   printFs();
            double computeMetropolisCriteria(const double t);
			void init();

			void setEpsilon(double epsilon){ epsilon_ = epsilon; }
			void setNEpsilon(unsigned nEpsilon){ nEpsilon_ = nEpsilon; }

        private:
            StaticComputationT& staticComputation_;

            double   fp_;
            double   f_;
            double   fOpt_;
            unsigned nEpsilon_;
            double   epsilon_;
            HybridWeightings hybridParameters_;
            std::string performanceCriterion_;

            std::vector<double>   fLatest_;
            std::vector<unsigned> dofsIndexListToCalibrate_;
        };
    }
}

#include "MinTorqueTrackEMGs.cpp"
#endif