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

#include <float.h>
#include <iostream>
using std::cout;
using std::endl;
#include <vector>
using std::vector;

#include <math.h>
#include "HybridWeightings.h"

#include <algorithm> // for max_element

namespace Hybrid {
    namespace ObjectiveFunction {
        
        template<typename StaticComputationT>
        MinTorqueTrackEMGs<StaticComputationT>::MinTorqueTrackEMGs(
                            StaticComputationT& staticComputation,
                            double epsilon, double nEpsilon, HybridWeightings hybridParameters, std::string performanceCriterion):
        epsilon_(epsilon),
        nEpsilon_(nEpsilon),
        staticComputation_(staticComputation),
        hybridParameters_(hybridParameters),
        performanceCriterion_(performanceCriterion),
        f_(DBL_MAX),fOpt_(DBL_MAX) {         }

		template<typename StaticComputationT>
		void MinTorqueTrackEMGs<StaticComputationT>::init()
		{
			fLatest_.resize(nEpsilon_);
			for (unsigned int a = 0; a < nEpsilon_; ++a)
				fLatest_.at(a) = DBL_MAX;
		}

        template<typename StaticComputationT>
        void MinTorqueTrackEMGs<StaticComputationT>::evalFp() {
        //1)
        //calculate first term of objective function: Least Squares Fitting
            
        double torqueLeastSquaresFitting = .0;
        vector<double> externalTorques, torques;
        staticComputation_.getExternalTorques(externalTorques);
        staticComputation_.getTorques(torques);

        for (unsigned d = 0; d < torques.size(); ++d)
        {
            torqueLeastSquaresFitting += fabs(externalTorques.at(d) - torques.at(d)) * fabs(externalTorques.at(d) - torques.at(d));
            //std::cout << "externalTorques / torques: \t " << externalTorques.at(d) << " \t " << torques.at(d) << "\n";
        }
        //std::cout << "torque error: " << torqueLeastSquaresFitting << "\n";
            //2)
            //calculate the second term of objective function: track experimental EMGs
            
            double emgTracking = .0;
            vector<double> experimentalEMGs, adjustedEMGs;
            staticComputation_.getInitialValuesOfTrackedEMGs(experimentalEMGs); //emg value for the tracked muscles before the emg adjustment
            staticComputation_.getAdjustedValuesOfTrackedEMGs(adjustedEMGs);
        
            /*cout << "\n\nExp vs Adj EMGS: " << experimentalEMGs.size() << "\n";
            for (unsigned k = 0; k < experimentalEMGs.size(); ++k) 
                cout << experimentalEMGs.at(k) << "\t" << adjustedEMGs.at(k) << endl;*/
        
            for(unsigned e = 0; e < adjustedEMGs.size(); ++e)
                emgTracking += fabs(experimentalEMGs.at(e) - adjustedEMGs.at(e));
            //std::cout << "EMG error: " << emgTracking << "\n";
            //3) Calculate the performance criterion term of the objective function
            double performanceCriterion = .0;
            if (performanceCriterion_ == "stress") // Option 1 (sum of relative muscle forces squared):
            {
                vector<double> currentForces;
                staticComputation_.getCurrentMuscleForcesRelative(currentForces);
                for (unsigned i = 0; i < currentForces.size(); ++i) {
                    performanceCriterion += currentForces.at(i) * currentForces.at(i);
                }
                //std::cout << "stress" << " ";
                performanceCriterion /= 18.; // to make the size of the value similar to the one for excitations
            }
            else if (performanceCriterion_ == "max_stress") // Option 2 (max relative muscle force):
            {
                vector<double> currentForces;
                staticComputation_.getCurrentMuscleForcesRelative(currentForces);
                performanceCriterion = *max_element(currentForces.begin(), currentForces.end());
                //std::cout << "max_stress" << " ";
                performanceCriterion /= 13.; // to make the size of the value similar to the one for excitations
            }
            else if (performanceCriterion_ == "energy") // Option 3 (energy-related cost function):
            {
                vector<double> currentEnergy;
                staticComputation_.getCurrentMuscleEnergy(currentEnergy);
                for (unsigned i = 0; i < currentEnergy.size(); ++i) {
                    performanceCriterion += currentEnergy.at(i);
                }
                //std::cout << "energy" << " ";
                performanceCriterion /= 22850.; // to make the size of the value similar to the one for excitations
            }
            else // default: "excitation" (sum of all EMGs squared)
            {
                vector<double> currentEMGs;
                //cout << "emgs in objective function \n";
                staticComputation_.getCurrentEMGs(currentEMGs);
                for (unsigned i = 0; i < currentEMGs.size(); ++i) {
                    performanceCriterion += currentEMGs.at(i) * currentEMGs.at(i);
                    //cout << currentEMGs.at(i) << " ";
                }
                //std::cout << "excitation" << " ";
            }

            //std::cout << performanceCriterion << std::endl;
            fp_ = hybridParameters_.alpha * torqueLeastSquaresFitting + hybridParameters_.gamma * emgTracking + hybridParameters_.beta * performanceCriterion;
            
            //cout << endl;
            //std::cout << "Excitations: " << emgSum << "\n";

  /*      cout << "torquesFitting " << torqueLeastSquaresFitting << endl;
        cout << "trackEMG_ " << emgTracking << endl;
        cout << "sumEMG_ " << emgSum << endl; */
        }


        template<typename StaticComputationT>
        bool MinTorqueTrackEMGs<StaticComputationT>::isFacceptable() {
        
            if ( fp_ > f_ )
                return false;
            return true;
        }


        template<typename StaticComputationT>
        void MinTorqueTrackEMGs<StaticComputationT>::updateF() {
            
            f_ = fp_;
        }



        template<typename StaticComputationT>
        bool MinTorqueTrackEMGs<StaticComputationT>::isFoptAcceptable() {
            
            if ( fp_ > fOpt_ )
                return false;
            return true;
        }


        template<typename StaticComputationT>
        void MinTorqueTrackEMGs<StaticComputationT>::updateFopt() {
            
            fOpt_ = fp_;
        }


        template<typename StaticComputationT>
        void MinTorqueTrackEMGs<StaticComputationT>::printFs() {

            cout << "fp_: ";
            cout << fp_ << " ";
            cout << endl;
            cout << "f_: ";
            cout << f_ << " ";
            cout << endl;
            cout << "fOpt_: ";
            cout << fOpt_ << " ";
            cout << endl; 
        }


        template<typename StaticComputationT>
        double MinTorqueTrackEMGs<StaticComputationT>::computeMetropolisCriteria(const double t) {

            double p = (f_ -fp_);
 //           cout << "f_ " << f_ << " fp_ " << fp_  << endl;
            return exp(p/t);
        }


        template<typename StaticComputationT>
        bool MinTorqueTrackEMGs<StaticComputationT>::terminate() {
            
            fLatest_.at(0) = f_;
            for (unsigned int a = 0; a < nEpsilon_; ++a) {
                if ( fabs( fLatest_.at(a) - fOpt_ ) > epsilon_ ) 
                    return false;
            }
            return true;
        }



        template<typename StaticComputationT>
        void MinTorqueTrackEMGs<StaticComputationT>::updateFandFlatest() {
            for (unsigned int a = 1; a < nEpsilon_; ++a) 
                fLatest_.at(nEpsilon_-a) = fLatest_.at(nEpsilon_-a-1);
            f_ = fOpt_;     
        }
        
    }
}
