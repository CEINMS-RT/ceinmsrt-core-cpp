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

#ifndef ErrorMinimizerAnnealing_h
#define ErrorMinimizerAnnealing_h

#include "HybridWeightings.h"

namespace Hybrid {
    
    template<typename NMSmodelT>
    class ErrorMinimizerAnnealing {
    
    public:
        ErrorMinimizerAnnealing(NMSmodelT& subject);  
        void setMusclesNamesWithEmgToTrack(const std::vector<std::string>& musclesNamesWithEmgToTrack);
        void setMusclesNamesWithEmgToPredict(const std::vector<std::string>& musclesNamesWithEmgToPredict);
        void setSingleExternalTorque(double externalTorque, const std::string& whichDof);
        void setAllExternalTorques(std::vector<double> externalTorques, const std::vector< std::string >& dofs);
        void setTime(double time) {currentTime_ = time;}
        void setWeightings(HybridWeightings weightings) {weightings_ = weightings;}
        void setPerformanceCriterion(const std::string performanceCriterion) { performanceCriterion_ = performanceCriterion; }
		void setNT(double nt){ nt_ = nt; }
		void setNS(double ns){ ns_ = ns; }
		void setRT(double rt){ rt_ = rt; }
		void setT(double t){ t_ = t; }
		void setMaxNoEval(int maxNoEval){ maxNoEval_ = maxNoEval; }
		void setEpsilon(double epsilon){ epsilon_ = epsilon; }
		void setNEpsilon(unsigned nEpsilon){ nEpsilon_ = nEpsilon; }
        void minimize();
    private:
        HybridWeightings weightings_;
        std::string performanceCriterion_;
        NMSmodelT& subject_;
        std::vector<double> currentExternalTorques_;
        std::vector<std::string> subjectDofNames_;
        std::vector<std::string> currentDofNames_;
        std::vector<std::string> musclesNamesWithEmgToTrack_;
        std::vector<std::string> musclesNamesWithEmgToPredict_;
        std::vector<double> lastActivations_;
        double currentTime_;
		double              nt_;
		double              ns_;
		double              rt_;
		double              t_;
		int                 maxNoEval_;
		double				epsilon_;
		unsigned			nEpsilon_;
    };
}

#include "ErrorMinimizerAnnealing.cpp"

#endif