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

namespace Hybrid {
    namespace StaticComputationMode {

        template<typename NMSmodelT>
        Default<NMSmodelT>::Default ( NMSmodelT& subject ):
        subject_(subject) {
            
            subject_.getEmgs(emgDataT1_);
			subject_.updateState_HYBRID();
			//std::cout << emgDataT1_[0] << std::endl;
           // musclesToUpdate_.assign(subject_.getNoMuscles(), true);
			//subject_.updateState();
		/*	subject_.updateActivations();
			subject_.updateFibreLengthsAndVelocities();
			subject_.updateMuscleForces();
			//subject_.updateTorques();
			subject_.pushState();

			subject_.getMuscleForces(forceDataT1_);
			cout << "Default.cpp muscle forces\n";
			for (unsigned i = 0; i < forceDataT1_.size(); ++i)
				cout << forceDataT1_.at(i) << " ";
			cout << endl;
			for (unsigned i = 0; i < emgDataT1_.size(); ++i)
				cout << emgDataT1_.at(i) << " ";
			cout << endl;*/
			
        }

        template<typename NMSmodelT>
        void Default<NMSmodelT>::getMusclesToUpdate() {
        
            musclesToUpdate_.clear();
            vector<double> currentEMGs;
            subject_.getEmgs(currentEMGs);
            for(unsigned idx = 0; idx < currentEMGs.size(); ++idx)
                if(currentEMGs.at(idx) != emgDataT1_.at(idx))
                    musclesToUpdate_.push_back(idx);            
            emgDataT1_ = currentEMGs;
        }


        
        template<typename NMSmodelT>
        void Default<NMSmodelT>::getTorques (vector< double >& torques) {

            getMusclesToUpdate();
            subject_.updateState_HYBRID(musclesToUpdate_);
            subject_.getTorques(torques);
        
//              std::cout << "currentTorques\n";
//             for (unsigned i = 0; i < torques.size(); ++i)
//                 std::cout << torques.at(i) << " ";
//             std::cout << std::endl;
            
            
     /*        vector<double> currentEMGs;
            
           std::cout << "currentEMGs\n";
            subject_.getEmgs(currentEMGs);
            for (unsigned i = 0; i < currentEMGs.size(); ++i)
                std::cout << currentEMGs.at(i) << " ";
            std::cout << std::endl;
            */
        }

    }
}