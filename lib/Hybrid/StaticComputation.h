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