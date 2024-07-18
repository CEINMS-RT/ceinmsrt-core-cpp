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




