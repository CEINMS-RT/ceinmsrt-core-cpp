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

#ifndef StaticComputation_h
#define StaticComputation_h

#include <vector>
#include "NMSmodel.h"
//#include "MuscleParameters.h" // to get muscle parameters (such as max isometric force)

namespace Hybrid {

    template<typename NMSmodelT, typename StaticComputationMode>
    class StaticComputation;

    template<typename NMSmodelT, typename StaticComputationMode>
    std::ostream& operator<< (std::ostream& output, const StaticComputation<NMSmodelT, StaticComputationMode>& m); 

    template<typename NMSmodelT, typename StaticComputationMode>
    class StaticComputation {

    public:
        StaticComputation(NMSmodelT& subject, 
                        const std::vector<std::string>& muscleNamesWithEMGtoTrack,
                        const std::vector<std::string>& muscleNamesWithEMGtoPredict);
        void setTime(double time) { time_ = time; computationMode_.setTime(time); }
        void setExternalTorques(const std::vector<double>& externalTorques, const std::vector<std::string>& dofNames);
        void getExternalTorques(std::vector<double>& externalTorques) const;
        void getTorques(std::vector<double>& torques);
        void getInitialValuesOfTrackedEMGs(std::vector<double>& emgValues) const;
        void getAdjustedValuesOfTrackedEMGs(std::vector<double>& emgValues) const;
        void getCurrentEMGs(std::vector<double>& emgValues) const;

        // To try different performance criteria of objective function:
        void getCurrentMuscleForcesRelative(std::vector<double>& muscleForces) const;
        void getCurrentMuscleEnergy(std::vector<double>& muscleEnergy) const;
        
    private:
        
        NMSmodelT& subject_;
        StaticComputationMode computationMode_;
        double time_;
        std::vector<double> externalTorques_;
        std::vector<double> initialTorques_;   //torques at the start of the frame, before optimization
        std::vector<double> initialValueOfTrackedEMGs_;
        std::vector<unsigned> muscleIndexWithEMGtoTrack_;
        std::vector<unsigned> muscleIndexWithEMGtoPredict_;
        std::vector<unsigned> muscleIndexWithEMGtoOptimize_;
        
        
    };
}

#include "StaticComputation.cpp"

#endif