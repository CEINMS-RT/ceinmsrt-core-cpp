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

#include "ExternalTorqueFromX.h"
#include "DataFromFile.h"
#include "SyncTools.h"

#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <cstdlib>

void ExternalTorqueFromX::pushExternalTorqueBack(const vector<double>& newExternalTorqueToPush)
{ 
//   SyncTools::Shared::queueExternalTorqueSemEmpty.wait();  //waits if the buffer is full. if so it waits until the consumer removes at least one item from queueLmt
//   SyncTools::Shared::queueExternalTorqueMutex.lock();
//   SyncTools::Shared::queueExternalTorque.at(whichDof).push_back(newExternalTorqueToPush);
//   SyncTools::Shared::queueExternalTorqueMutex.unlock();
//   SyncTools::Shared::queueExternalTorqueSemFull.notify(); //notify that an item has been pushed in the queue
	InterThread::setExternalTorque(newExternalTorqueToPush);
}



void ExternalTorqueFromX::updateExternalTorque(const vector<double>& currentExternalTorqueData, double currentTime)
{
  vector<double> ExternalTorqueDataToPush = currentExternalTorqueData;
  ExternalTorqueDataToPush.push_back(currentTime); //appends currentTime at the end
  pushExternalTorqueBack(ExternalTorqueDataToPush);
 
}



//TODO: i nomi dei dof devono essere un sottoinsieme dei dof dall'xml
void ExternalTorqueFromX::setExternalTorqueDofNames(vector<string> dofNamesWithExtTorqueFromInput)
{
//   SyncTools::Shared::dofNamesWithExtTorqueMutex.lock();
//   SyncTools::Shared::dofNamesWithExtTorque = dofNamesWithExtTorqueFromInput;
//   SyncTools::Shared::dofNamesWithExtTorqueMutex.unlock();

//TODO: forse non è il posto migliore per fare quello che c'è sotto..mmmm
//   SyncTools::Shared::queueExternalTorqueMutex.lock();
//   SyncTools::Shared::queueExternalTorque.resize(dofNamesWithExtTorqueFromInput.size());  
//   SyncTools::Shared::queueExternalTorqueMutex.unlock();
}

void ExternalTorqueFromX::getDofNames(vector<string>& dofNamesFromModel)
{
//   SyncTools::Shared::dofNamesSem.wait();
//   dofNamesFromModel = SyncTools::Shared::dofNames; //gets dof names from XML model, passed from global variable dofNames  
//   SyncTools::Shared::dofNamesSem.notify();
	dofNamesFromModel = InterThread::getDofNames();
}  

ExternalTorqueFromX::~ExternalTorqueFromX() { }




