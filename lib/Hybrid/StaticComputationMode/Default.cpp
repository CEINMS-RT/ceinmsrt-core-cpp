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