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

#include "LmtMaFromX.h"
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

void LmtMaFromX::pushLmtBack(const vector<double>& newLmtToPush)
{ 
//   SyncTools::Shared::queueLmtSemEmpty.wait();  //waits if the buffer is full. if so it waits until the consumer removes at least one item from queueLmt
//   SyncTools::Shared::queueLmtMutex.lock();
//   SyncTools::Shared::queueLmt.push_back(newLmtToPush);
//   SyncTools::Shared::queueLmtMutex.unlock();
//   SyncTools::Shared::queueLmtSemFull.notify(); //notify that an item has been pushed in the queue
	InterThread::setLMT(newLmtToPush);
}

void LmtMaFromX::pushMomentArmsBack(const vector<double>& newMomentArmsToPush, unsigned int whichDof)
{ 
//   SyncTools::Shared::queueMomentArmsSemEmpty.wait(); //waits if the buffer is full. if so it waits until the consumer removes at least one item from queueMomentArms
//   SyncTools::Shared::queueMomentArmsMutex.lock();
//   SyncTools::Shared::queueMomentArms.at(whichDof).push_back(newMomentArmsToPush);
//   SyncTools::Shared::queueMomentArmsMutex.unlock();
//   SyncTools::Shared::queueMomentArmsSemFull.notify(); //notify that an item has been pushed in the queue
	InterThread::setMomentsArm(newMomentArmsToPush, whichDof);
}

void LmtMaFromX::updateLmt(const vector<double>& currentLmtData, double currentTime)
{
  vector<double> lmtDataToPush = currentLmtData;
  lmtDataToPush.push_back(currentTime); //appends currentTime at the end
  pushLmtBack(lmtDataToPush);
}

void LmtMaFromX::updateMomentArms(const vector<double>& currentMomentArmsData, double currentTime, unsigned int whichDof)
{
  vector<double> momentArmsDataToPush = currentMomentArmsData;
  momentArmsDataToPush.push_back(currentTime); //appends currentTime at the end
  pushMomentArmsBack(momentArmsDataToPush, whichDof);
}
 
void LmtMaFromX::getDofNames(vector<string>& dofNamesFromModel)
{
//   SyncTools::Shared::dofNamesSem.wait();
//   dofNamesFromModel = SyncTools::Shared::dofNames; //gets dof names from XML model, passed from global variable dofNames  
//   SyncTools::Shared::dofNamesSem.notify();
// 
// //TODO: forse non è il posto migliore per fare quello che c'è sotto..mmmm
//   SyncTools::Shared::queueMomentArmsMutex.lock();
//   SyncTools::Shared::queueMomentArms.resize(dofNamesFromModel.size());  
//   SyncTools::Shared::queueMomentArmsMutex.unlock();
	dofNamesFromModel = InterThread::getDofNames();
}

void LmtMaFromX::getMuscleNamesFromShared(std::vector< std::string >& muscleNamesFromModel)
{
// 	  SyncTools::Shared::musclesNamesSem.wait();
// 	  muscleNamesFromModel = SyncTools::Shared::musclesNames; //gets dof names from XML model, passed from global variable dofNames
// 	  SyncTools::Shared::musclesNamesSem.notify();
	muscleNamesFromModel = InterThread::getMusclesNames();
}

void LmtMaFromX::getAngleFromShared(std::vector<double>& angleFromQueue)
{
// 	// The modelEvaluation will pop the value.
// 	SyncTools::Shared::queueAngleSemFull.wait();
//     SyncTools::Shared::queueAngleMutex.lock();
// 
//     angleFromQueue = SyncTools::Shared::queueAngle.front();
// 
//     SyncTools::Shared::queueAngleMutex.unlock();
//     SyncTools::Shared::queueAngleSemFull.notify();
	angleFromQueue = InterThread::getAngle();
}

void LmtMaFromX::setMusclesNamesOnDof(std::vector<std::vector<std::string> >& musclesNamesOnDof)
{
// 	SyncTools::Shared::musclesNamesOnDofMutex.lock();
// 	SyncTools::Shared::musclesNamesOnDof = musclesNamesOnDof_;
// 	SyncTools::Shared::musclesNamesOnDofMutex.unlock();
	
	InterThread::setMusclesNamesOnDof(musclesNamesOnDof);
}

void LmtMaFromX::getSyncGui(bool& gui)
{
	gui = InterThread::getSyncGui();
}

void LmtMaFromX::getSyncVerbose(int& verbose)
{
	verbose = InterThread::getSyncVerbose();
}

void AddToTimeConsumeMTU(const double& timeToAdd)
{
	InterThread::AddToTimeConsumeMTU(timeToAdd);
}

void AddToNbFrameMTU(const double& timeToAdd)
{
	InterThread::AddToNbFrameMTU((const int)timeToAdd);
}
  
LmtMaFromX::~LmtMaFromX() { }




