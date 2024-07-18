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
#ifndef DataSynchronizer_h
#define DataSynchronizer_h

#include <vector>
#include <list>
#include <limits>
#include <algorithm>

#include <iostream>
#include <stdlib.h>

#include "SyncTools.h"

namespace SyncTools {
    
    
    class DataSynchronizer {
    public:
        void getEmgs(std::vector<double>& emgs);
        void getMuscleTendonLengths(std::vector<double>& lmt);
        void getMomentArms(std::vector<double>& ma, unsigned whichDof);
        void getTorques(std::vector<double>& torques);
        void hasTorques(bool hasTorques) { hasTorques_ = hasTorques;}
        bool hasTorques() { return hasTorques_; }
        
    private:
        void popEmgFront(std::vector<double>& emgFromQueue);
        void popLmtFront(std::vector<double>& lmtFromQueue);
        void popMomentArmsFront(std::vector<double>& momentArmsFromQueue, unsigned int whichDof);
        void popExternalTorqueFront(std::vector<double>& externalTorqueFromQueue, unsigned int whichDof);   

        bool hasTorques_;
    };
    
    
void DataSynchronizer::getEmgs(std::vector<double>& emgs) {

    
}

    

    void DataSynchronizer::popEmgFront(std::vector<double>& emgFromQueue) {   
        
        Shared::queueEmgSemFull.wait(); //waits if there is no item in queueEmg
        Shared::queueEmgMutex.lock();   

        emgFromQueue = Shared::queueEmg.front(); 
        Shared::queueEmg.pop_front(); 
        
        Shared::queueEmgMutex.unlock();
        Shared::queueEmgSemEmpty.notify();  //notify that an item has been removed from queueEmg
                                    
    }

    void DataSynchronizer::popLmtFront(std::vector<double>& lmtFromQueue) { 

        Shared::queueLmtSemFull.wait();
        Shared::queueLmtMutex.lock(); 
 
        lmtFromQueue = Shared::queueLmt.front(); 
        Shared::queueLmt.pop_front();  
  
        Shared::queueLmtMutex.unlock();
        Shared::queueLmtSemEmpty.notify();
 
    }

    void DataSynchronizer::popMomentArmsFront(std::vector<double>& momentArmsFromQueue, unsigned int whichDof) { 
  
        Shared::queueMomentArmsSemFull.wait();
        Shared::queueMomentArmsMutex.lock();   

        momentArmsFromQueue = Shared::queueMomentArms.at(whichDof).front(); 
        Shared::queueMomentArms.at(whichDof).pop_front();  

        Shared::queueMomentArmsMutex.unlock();
        Shared::queueMomentArmsSemEmpty.notify();
    }
    
  
    void DataSynchronizer::popExternalTorqueFront(vector<double>& externalTorqueFromQueue, unsigned int whichDof) {   
    
        Shared::queueExternalTorqueSemFull.wait(); //waits if there is no item in queue
        Shared::queueExternalTorqueMutex.lock();   

        externalTorqueFromQueue = (Shared::queueExternalTorque.at(whichDof)).front(); 
        Shared::queueExternalTorque.at(whichDof).pop_front(); 
  
        Shared::queueExternalTorqueMutex.unlock();
        Shared::queueExternalTorqueSemEmpty.notify();  //notify that an item has been removed from queue
    }
};


#endif