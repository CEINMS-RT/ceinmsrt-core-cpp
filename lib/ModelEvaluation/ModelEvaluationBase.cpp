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




