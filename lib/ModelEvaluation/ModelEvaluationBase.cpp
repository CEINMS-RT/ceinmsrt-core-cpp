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

#include "ModelEvaluationBase.h"
#include "SyncTools.h"

#include <vector>
using std::vector;
#include <string>
using std::string;

void ModelEvaluationBase::getEmgFromShared ( vector<double>& emgs )
{

//     SyncTools::Shared::queueEmgSemFull.wait(); //waits if there is no item in queueEmg
//     SyncTools::Shared::queueEmgMutex.lock();
//
//     emgs = SyncTools::Shared::queueEmg.front();
//     SyncTools::Shared::queueEmg.pop_front();
//
//     SyncTools::Shared::queueEmgMutex.unlock();
//     SyncTools::Shared::queueEmgSemEmpty.notify();  //notify that an item has been removed from queueEmg
	emgs = InterThread::getEMG();
}

void ModelEvaluationBase::getLmtFromShared ( vector<double>& lmts )
{

//     SyncTools::Shared::queueLmtSemFull.wait();
//     SyncTools::Shared::queueLmtMutex.lock();
//
//     lmts = SyncTools::Shared::queueLmt.front();
//     SyncTools::Shared::queueLmt.pop_front();
//
//     SyncTools::Shared::queueLmtMutex.unlock();
//     SyncTools::Shared::queueLmtSemEmpty.notify();
	lmts = InterThread::getLMT();
}

void ModelEvaluationBase::getAngleFromShared ( std::vector<double>& angleFromQueue )
{
// 	SyncTools::Shared::queueAngleSemFull.wait();
//     SyncTools::Shared::queueAngleMutex.lock();
//
//     angleFromQueue = SyncTools::Shared::queueAngle.front();
//     SyncTools::Shared::queueAngle.pop_front();
//
//     SyncTools::Shared::queueAngleMutex.unlock();
//     SyncTools::Shared::queueAngleSemEmpty.notify();
	angleFromQueue = InterThread::getAngle();
}

void ModelEvaluationBase::getMomentArmsFromShared ( vector<double>& momentArms, unsigned int whichDof )
{
//     SyncTools::Shared::queueMomentArmsSemFull.wait();
//     SyncTools::Shared::queueMomentArmsMutex.lock();
//
//     momentArms = SyncTools::Shared::queueMomentArms.at(whichDof).front();
//     SyncTools::Shared::queueMomentArms.at(whichDof).pop_front();
//
//     SyncTools::Shared::queueMomentArmsMutex.unlock();
//     SyncTools::Shared::queueMomentArmsSemEmpty.notify();
	momentArms = InterThread::getMomentsArm ( whichDof );
}


void ModelEvaluationBase::getExternalTorqueFromShared ( vector<double>& externalTorque)
{

//     SyncTools::Shared::queueExternalTorqueSemFull.wait(); //waits if there is no item in queue
//     SyncTools::Shared::queueExternalTorqueMutex.lock();
//
//     externalTorque = SyncTools::Shared::queueExternalTorque.at(whichDof).front();
//     SyncTools::Shared::queueExternalTorque.at(whichDof).pop_front();
//
//     SyncTools::Shared::queueExternalTorqueMutex.unlock();
//     SyncTools::Shared::queueExternalTorqueSemEmpty.notify();  //notify that an item has been removed from queue
	externalTorque = InterThread::getExternalTorque (  );
}


void ModelEvaluationBase::getDofNamesAssociatedToExternalTorque ( vector<string>& dofNames )
{

//     SyncTools::Shared::dofNamesWithExtTorqueMutex.lock();
//     dofNames = SyncTools::Shared::dofNamesWithExtTorque; //make a local copy of global variable dofNamesWithExtTorque
//     SyncTools::Shared::dofNamesWithExtTorqueMutex.unlock();
}


void ModelEvaluationBase::getMusclesNamesFromShared ( vector<string>& muscleNames )
{

//     SyncTools::Shared::musclesNamesMutex.lock();
//     muscleNames = SyncTools::Shared::musclesNames; //make a local copy of global variable musclesNames
//     SyncTools::Shared::musclesNamesMutex.unlock();
	muscleNames = InterThread::getMusclesNames();
}

void ModelEvaluationBase::setMuscleNamesToShared ( const vector<string>& muscleNames )
{
// 	SyncTools::Shared::musclesNamesMutex.lock();
// 	SyncTools::Shared::musclesNames = muscleNames;
// 	SyncTools::Shared::musclesNamesMutex.unlock();
// 	SyncTools::Shared::musclesNamesSem.notify();
	InterThread::setMusclesNames ( muscleNames );
}

void ModelEvaluationBase::getMusclesNamesOnDofsFromShared ( vector< vector<string> >& muscleNamesOnDofs )
{
//     SyncTools::Shared::musclesNamesOnDofMutex.lock();
//     muscleNamesOnDofs = SyncTools::Shared::musclesNamesOnDof; //make a local copy of global variable musclesNamesOnDof
//     SyncTools::Shared::musclesNamesOnDofMutex.unlock();
	muscleNamesOnDofs = InterThread::getMusclesNamesOnDof (  );
}


//controllare se viene usata
void ModelEvaluationBase::setDofNamesToShared ( const vector<string>& dofNames )
{
//     SyncTools::Shared::dofNames = dofNames; //dofNames is a global variable, it's needed for LmtMaFromFile class
//     SyncTools::Shared::dofNamesSem.notify();
	InterThread::setDofNames(dofNames);
}

void ModelEvaluationBase::setMusclesNamesOnChannel ( std::map<std::string, std::vector <std::string> > musclesNamesOnChannel )
{
// 	SyncTools::Shared::musclesNamesOnChannel = musclesNamesOnChannel;
// 	SyncTools::Shared::musclesNamesOnChannelSem.notify();
	InterThread::setMusclesNamesOnChannel ( musclesNamesOnChannel );
}

void ModelEvaluationBase::getSyncGui(bool& gui)
{
	gui = InterThread::getSyncGui();
}

void ModelEvaluationBase::getSyncVerbose(int& verbose)
{
	verbose = InterThread::getSyncVerbose();
}

ModelEvaluationBase::~ModelEvaluationBase() { }




