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