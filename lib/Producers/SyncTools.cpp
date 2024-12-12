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

#include "SyncTools.h"

namespace SyncTools
{
	namespace Shared
	{
		float globalTimeLimit = 100.;
		const int queueBuffer = 10000;
		const unsigned int numberOfThreads = 2; // 4 se c'è il produttore della torque esterna, 3 altrimenti

		// 		list< vector<double> > queueEmg;
		// 		boost::mutex queueEmgMutex;
		// 		Semaphore queueEmgSemFull ( 0 ), queueEmgSemEmpty ( queueBuffer );
		//
		// 		list< vector<double> > queueLmt;
		// 		boost::mutex queueLmtMutex;
		// 		Semaphore queueLmtSemFull ( 0 ), queueLmtSemEmpty ( queueBuffer );
		//
		// 		vector< list< vector<double> > > queueMomentArms;
		// 		boost::mutex queueMomentArmsMutex;
		// 		Semaphore queueMomentArmsSemFull ( 0 ), queueMomentArmsSemEmpty ( queueBuffer );
		//
		// 		vector< list< vector <double> > > queueExternalTorque;
		// 		boost::mutex queueExternalTorqueMutex;
		// 		Semaphore queueExternalTorqueSemFull ( 0 ), queueExternalTorqueSemEmpty ( queueBuffer );
		//
		// 		list< vector <double> > queueAngle;
		// 		boost::mutex queueAngleMutex;
		// 		Semaphore queueAngleSemFull ( 0 ), queueAngleSemEmpty ( queueBuffer );
		//
		//
		// 		//EMGgenerator provides the names, emg and lmt files must have corresponding headers
		// 		vector<string> musclesNames;
		// 		Semaphore musclesNamesSem ( 0 );
		// 		boost::mutex musclesNamesMutex;
		//
		// 		//XML model provides a list of muscles names for each dof, corresponding moment arms files must have the same set of muscles names
		// 		vector< vector <string> > musclesNamesOnDof;
		// 		boost::mutex musclesNamesOnDofMutex;
		//
		// 		vector<string> dofNamesWithExtTorque;
		// 		boost::mutex dofNamesWithExtTorqueMutex;
		//
		// 		vector<string> dofNamesWithExtAngle;
		// 		boost::mutex dofNamesWithExtAngleMutex;
		//
		// 		bool endThread;
		// 		boost::mutex endThreadMutex;
		//
		// 		double lmtTime;
		// 		boost::mutex lmtTimeMutex;
		//
		// 		double EMGTime;
		// 		boost::mutex EMGTimeMutex;
		//
		// 		string libPathEMG;
		// 		boost::mutex libPathEMGMutex;
		//
		// 		string libPathAngle;
		// 		boost::mutex libPathAngleMutex;
		//
		// 		map<string, vector <string> > musclesNamesOnChannel;
		// 		Semaphore musclesNamesOnChannelSem ( 0 );
		//
		// 		vector<string> dofNames;
		// 		Semaphore dofNamesSem ( 0 );
		//
		// 		Semaphore lmtProducingDone ( 0 );
		//
		// 		Semaphore emgProducingDone ( 0 );
		//
		// 		Semaphore consumerDone ( 0 );
		// 		Semaphore NMSconsumerDoneEMG ( 0 );
		// 		Semaphore NMSconsumerDoneAngle ( 0 );
		//
		// 		Semaphore angleProducingDone ( 0 );
		//
		//
		// 		boost::barrier readyToStart ( numberOfThreads );
		//
		//
		// 		vector <double> muscleForceGui, timeEmg, timeTorque;
		// 		std::vector<std::vector<double> > torqueGui, emgGui, idGui;
		// 		double timeConsumeNMS, timeEndNMS, timeConsumeMTU;
		// 		int nbFrameNMS, nbFrameMTU, nbFrameEMG;
		//
		// 		boost::mutex mutexGui;
		//
		// 		std::string modelFileName;
		// 		Semaphore semModelFileName ( 0 );
		//
		// 		std::map<std::string, double> positionMap;
		// 		boost::mutex positionMapMutex;
		//
		// 		boost::mutex syncGuiMutex;
		// 		bool syncGui;
		//
		// 		boost::mutex syncVerboseMutex;
		// 		int syncVerbose;
	};
};

void InterThread::notifyAll()
{
	lmtProducingDone.notify();
	emgProducingDone.notify();
	angleProducingDone.notify();
	consumerDone.notify();
	NMSconsumerDoneEMG.notify();
	NMSconsumerDoneAngle.notify();
	_queueAngleSemFull.notify();
	_queueAngleSemEmpty.notify();
	_queueLmtSemEmpty.notify();
	_queueLmtSemFull.notify();
	_queueEmgSemFull.notify();
	_queueEmgSemEmpty.notify();
	_queueMomentArmsSemFull.notify();
	_queueMomentArmsSemEmpty.notify();
	_queueExternalTorqueSemFull.notify();
	_queueExternalTorqueSemEmpty.notify();
}

bool InterThread::getEndThread()
{
	boost::lock_guard<boost::mutex> lock(_endThreadMutex);
	return _endThread;
}

void InterThread::setEndThread(const bool& end)
{
	boost::lock_guard<boost::mutex> lock(_endThreadMutex);
	_endThread = end;
}

bool InterThread::getSyncGui()
{
	_syncGuiSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_syncGuiMutex);
	return _syncGui;
}

void InterThread::setSyncGui(const bool& syncGui)
{
	boost::lock_guard<boost::mutex> lock(_syncGuiMutex);
	_syncGui = syncGui;
	_syncGuiSem.notify();
}

int InterThread::getSyncVerbose()
{
	_syncVerboseSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_syncVerboseMutex);
	return _syncVerbose;
}

void InterThread::setSyncVerbose(const int& syncVerbose)
{
	boost::lock_guard<boost::mutex> lock(_syncVerboseMutex);
	_syncVerbose = syncVerbose;
	_syncVerboseSem.notify();
}

std::string InterThread::getLibPathEMG()
{
	_libPathEMGSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_libPathEMGMutex);
	return _libPathEMG;
}

void InterThread::setLibPathEMG(const std::string& libPathEMG)
{
	boost::lock_guard<boost::mutex> lock(_libPathEMGMutex);
	_libPathEMG = libPathEMG;
	_libPathEMGSem.notify();
}

std::string InterThread::getLibPathAngleMutex()
{
	_libPathAngleSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_libPathAngleMutex);
	return _libPathAngle;
}

void InterThread::setLibPathAngleMutex(const std::string& libPathAngleMutex)
{
	boost::lock_guard<boost::mutex> lock(_libPathAngleMutex);
	_libPathAngle = libPathAngleMutex;
	_libPathAngleSem.notify();
}

std::string InterThread::getModelFileName()
{
	_semModelFileName.waitOnce();
	boost::lock_guard<boost::mutex> lock(_modelFileNameMutex);
	return _modelFileName;
}

void InterThread::setModelFileName(const std::string& modelFileName)
{
	boost::lock_guard<boost::mutex> lock(_modelFileNameMutex);
	_modelFileName = modelFileName;
	_semModelFileName.notify();
}

double InterThread::getEMGTime()
{
	boost::lock_guard<boost::mutex> lock(_EMGTimeMutex);
	return _EMGTime;
}

void InterThread::setEMGTime(const double& emgTime)
{
	boost::lock_guard<boost::mutex> lock(_EMGTimeMutex);
	_EMGTime = emgTime;
}

double InterThread::getLmtTime()
{
	boost::lock_guard<boost::mutex> lock(_lmtTimeMutex);
	return _lmtTime;
}

void InterThread::setLmtTime(const double& lmtTime)
{
	boost::lock_guard<boost::mutex> lock(_lmtTimeMutex);
	_lmtTime = lmtTime;
	InterThread::setGuiTimePos(lmtTime);
}

std::vector<double> InterThread::getEMG()
{
	_queueEmgSemFull.wait();
	boost::lock_guard<boost::mutex> lock(_queueEmgMutex);
	std::vector<double> emg = _queueEmg.front();
	if (_queueEmg.size() != 0)
		_queueEmg.pop_front();
	_queueEmgSemEmpty.notify();
	return emg;
}

void InterThread::setEMG(const std::vector<double>& emg)
{
	_queueEmgSemEmpty.wait();
	boost::lock_guard<boost::mutex> lock(_queueEmgMutex);
	_queueEmg.push_back(emg);
	if (_queueEmg.size() > SyncTools::Shared::queueBuffer)
		_queueEmg.pop_front();
	_queueEmgSemFull.notify();
}

std::vector<double> InterThread::getLMT()
{
	_queueLmtSemFull.wait();
	boost::lock_guard<boost::mutex> lock(_queueLmtMutex);
	std::vector<double> lmt = _queueLmt.front();
	if (_queueLmt.size() != 0)
		_queueLmt.pop_front();
	_queueLmtSemEmpty.notify();
	return lmt;
}

bool InterThread::isEmptyLMT()
{
	boost::lock_guard<boost::mutex> lock(_queueLmtMutex);
	if (_queueLmt.size() != 0)
		return false;
	else
		return true;
}

void InterThread::setLMT(const std::vector<double>& lmt)
{
	_queueLmtSemEmpty.wait();
	boost::lock_guard<boost::mutex> lock(_queueLmtMutex);
	_queueLmt.push_back(lmt);
	if (_queueLmt.size() > SyncTools::Shared::queueBuffer)
		_queueLmt.pop_front();
	_queueLmtSemFull.notify();
}

std::vector<double> InterThread::getOptimaFiberLength()
{
	_OFLSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_OFLMutex);
	std::vector<double> OFL = _OFL;
	return OFL;
}

void InterThread::setOptimaFiberLength(const std::vector<double>& OFL)
{
	boost::lock_guard<boost::mutex> lock(_OFLMutex);
	_OFL = OFL;
	_OFLSem.notify();
}

std::vector<double> InterThread::getIsometricsMaxForce()
{
	_isometricsMaxForceSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_isometricsMaxForceMutex);
	std::vector<double> isometricsMaxForce = _isometricsMaxForce;
	return isometricsMaxForce;
}

void InterThread::setIsometricsMaxForce(const std::vector<double>& isometricsMaxForce)
{
	boost::lock_guard<boost::mutex> lock(_isometricsMaxForceMutex);
	_isometricsMaxForce = isometricsMaxForce;
	_isometricsMaxForceSem.notify();
}

std::vector<double> InterThread::getAngle()
{
	_queueAngleSemFull.wait();
	boost::lock_guard<boost::mutex> lock(_queueAngleMutex);
	std::vector<double> angle = _queueAngle.front();
	if (_queueAngle.size() != 0)
		_queueAngle.pop_front();
	_queueAngleSemEmpty.notify();
	return angle;
}

void InterThread::setAngle(const std::vector<double>& angle)
{
	//_queueAngleSemEmpty.wait();
	boost::lock_guard<boost::mutex> lock(_queueAngleMutex);
	_queueAngle.push_back(angle);
	if (_queueAngle.size() > SyncTools::Shared::queueBuffer)
		_queueAngle.pop_front();
	_queueAngleSemFull.notify();
}

std::vector<double> InterThread::getMomentsArm(unsigned int whichDof)
{
	_queueMomentArmsSemFull.wait();
	boost::lock_guard<boost::mutex> lock(_queueMomentArmsMutex);
	std::vector<double> momentsArm = _queueMomentArms.at(whichDof).front();
	if (_queueMomentArms.at(whichDof).size() != 0)
		_queueMomentArms.at(whichDof).pop_front();
	_queueMomentArmsSemEmpty.notify();
	return momentsArm;
}

void InterThread::setMomentsArm(const std::vector<double>& momentsArm, unsigned int whichDof)
{
	_queueMomentArmsSemEmpty.wait();
	boost::lock_guard<boost::mutex> lock(_queueMomentArmsMutex);
	_queueMomentArms.at(whichDof).push_back(momentsArm);
	if (_queueMomentArms.at(whichDof).size() > SyncTools::Shared::queueBuffer)
		_queueMomentArms.at(whichDof).pop_front();
	_queueMomentArmsSemFull.notify();
}

std::vector<double> InterThread::getExternalTorque()
{
	_queueExternalTorqueSemFull.wait();
	boost::lock_guard<boost::mutex> lock(_queueExternalTorqueMutex);
	std::vector<double> externalTorque = _queueExternalTorque.back(); // was front() for Open-Loop CEINMS, but changed to back() for Hybrid plugin
	if (_queueExternalTorque.size() != 0)
		_queueExternalTorque.pop_front();
	_queueExternalTorqueSemEmpty.notify();
	return externalTorque;
}

void InterThread::setExternalTorque(const std::vector<double>& externalTorque)
{
	//_queueExternalTorqueSemEmpty.wait();
	boost::lock_guard<boost::mutex> lock(_queueExternalTorqueMutex);
	_queueExternalTorque.push_back(externalTorque);
	if (_queueExternalTorque.size() > SyncTools::Shared::queueBuffer)
		_queueExternalTorque.pop_front();
	_queueExternalTorqueSemFull.notify();
}

std::vector< std::vector < std::string > > InterThread::getMusclesNamesOnDof()
{
	_musclesNamesOnDofSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_musclesNamesOnDofMutex);
	return _musclesNamesOnDof;
}

void InterThread::setMusclesNamesOnDof(const std::vector< std::vector < std::string > >& musclesNamesOnDof)
{
	boost::lock_guard<boost::mutex> lock(_musclesNamesOnDofMutex);
	_musclesNamesOnDof = musclesNamesOnDof;
	_musclesNamesOnDofSem.notify();
}

std::map<std::string, std::vector <std::string> > InterThread::getMusclesNamesOnChannel()
{
	_musclesNamesOnChannelSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_musclesNamesOnChannelMutex);
	return _musclesNamesOnChannel;
}

void InterThread::setMusclesNamesOnChannel(const std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel)
{
	boost::lock_guard<boost::mutex> lock(_musclesNamesOnChannelMutex);
	_musclesNamesOnChannel = musclesNamesOnChannel;
	_musclesNamesOnChannelSem.notify();
}

std::vector< std::string > InterThread::getMusclesNames()
{
	_musclesNamesSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_musclesNamesMutex);
	return _musclesNames;
}

void InterThread::setMusclesNames(const std::vector< std::string >& musclesNames)
{
	boost::lock_guard<boost::mutex> lock(_musclesNamesMutex);
	_musclesNames = musclesNames;
	_musclesNamesSem.notify();
}

std::vector< std::string > InterThread::getDofNames()
{
	_dofNamesSem.waitOnce();
	boost::lock_guard<boost::mutex> lock(_dofNamesMutex);
	return _dofNames;
}

void InterThread::setDofNames(const std::vector< std::string >& dofNames)
{
	boost::lock_guard<boost::mutex> lock(_dofNamesMutex);
	_dofNames = dofNames;
	_dofNamesSem.notify();
	boost::lock_guard<boost::mutex> lock2(_queueMomentArmsMutex);
	_queueMomentArms.resize(_dofNames.size());
}

std::map<std::string, double> InterThread::getPositionMap()
{
	boost::lock_guard<boost::mutex> lock(_positionMapMutex);
	return _positionMap;
}

void InterThread::setPositionMap(const std::map<std::string, double>& postionMap)
{
	boost::lock_guard<boost::mutex> lock(_positionMapMutex);
	_positionMap = postionMap;
}

std::vector<double> InterThread::getGuiTimeEmg()
{
	boost::lock_guard<boost::mutex> lock(_guiTimeEmgMutex);
	std::vector<double> temp = _guiTimeEmg;
	_guiTimeEmg.clear();
	return temp;
}

void InterThread::setGuiTimeEmg(const double& guiTimeEmg)
{
	boost::lock_guard<boost::mutex> lock(_guiTimeEmgMutex);
	_guiTimeEmg.push_back(guiTimeEmg);
}

std::vector<double> InterThread::getGuiTimeLMT()
{
	boost::lock_guard<boost::mutex> lock(_guiTimeLMTMutex);
	std::vector<double> temp = _guiTimeLMT;
	_guiTimeLMT.clear();
	return temp;
}

void InterThread::setGuiTimeLMT(const double& guiTimeLMT)
{
	boost::lock_guard<boost::mutex> lock(_guiTimeLMTMutex);
	_guiTimeLMT.push_back(guiTimeLMT);
}

std::vector<double> InterThread::getGuiTimePos()
{
	boost::lock_guard<boost::mutex> lock(_guiTimePosMutex);
	std::vector<double> temp = _guiTimePos;
	_guiTimePos.clear();
	return temp;
}

void InterThread::setGuiTimePos(const double& guiTimePos)
{
	boost::lock_guard<boost::mutex> lock(_guiTimePosMutex);
	_guiTimePos.push_back(guiTimePos);
}

std::vector<double> InterThread::getGuiMuscleForce()
{
	boost::lock_guard<boost::mutex> lock(_guiMuscleForceMutex);
	return _guiMuscleForce;
}

void InterThread::setGuiMuscleForce(const std::vector <double>& guiMuscleForce)
{
	boost::lock_guard<boost::mutex> lock(_guiMuscleForceMutex);
	_guiMuscleForce = guiMuscleForce;
}

std::vector <double> InterThread::getGuiTimeTorque()
{
	boost::lock_guard<boost::mutex> lock(_guiTimeTorqueMutex);
	std::vector <double> temp = _guiTimeTorque;
	_guiTimeTorque.clear();
	return temp;
}

void InterThread::setGuiTimeTorque(const double& guiTimeTorque)
{
	boost::lock_guard<boost::mutex> lock(_guiTimeTorqueMutex);
	_guiTimeTorque.push_back(guiTimeTorque);
}

void InterThread::getGuiEMG(std::vector<std::vector<double> >& guiEMG, std::vector<double>& guiTimeEmg)
{
	boost::lock_guard<boost::mutex> lock(_guiEMGMutex);
	guiEMG = _guiEMG;
	_guiEMG.clear(); 
	guiTimeEmg = _guiTimeEmg;
	_guiTimeEmg.clear();
}

void InterThread::setGuiEMG(const std::vector<double>& guiEMG, const double& guiTimeEmg)
{
	boost::lock_guard<boost::mutex> lock(_guiEMGMutex);
	_guiEMG.push_back(guiEMG);
	_guiTimeEmg.push_back(guiTimeEmg);
}


void InterThread::getGuiWorkLoop(std::vector<std::vector<double> >& guiForce, std::vector<std::vector<double> >& guiFiberLength, std::vector<double>& guiTimeWorkLoop)
{
	boost::lock_guard<boost::mutex> lock(_guiWorkLoopMutex);
	guiForce = _guiForce;
	_guiForce.clear(); 
	guiFiberLength = _guiFiberLength;
	_guiFiberLength.clear();
	guiTimeWorkLoop = _guiTimeWorkLoop;
	_guiTimeWorkLoop.clear();
}

void InterThread::setGuiWorkLoop(const std::vector<double>& guiForce, const std::vector<double>& guiFiberLength, const double& guiTimeWorkLoop)
{
	boost::lock_guard<boost::mutex> lock(_guiWorkLoopMutex);
	_guiTimeWorkLoop.push_back(guiTimeWorkLoop);
	_guiFiberLength.push_back(guiFiberLength);
	_guiForce.push_back(guiForce);
}

std::vector<std::vector<double> > InterThread::getGuiID()
{
	boost::lock_guard<boost::mutex> lock(_guiIDMutex);
	std::vector<std::vector<double> > temp = _guiID;
	_guiID.clear();
	return temp;
}

void InterThread::setGuiID(const std::vector<double>& guiID)
{
	boost::lock_guard<boost::mutex> lock(_guiIDMutex);
	_guiID.push_back(guiID);
}

std::vector<std::vector<double> > InterThread::getGuiTorque()
{
	boost::lock_guard<boost::mutex> lock(_guiTorqueMutex);
	std::vector<std::vector<double> > temp = _guiTorque;
	_guiTorque.clear();
	return temp;
}

void InterThread::setGuiTorque(const std::vector<double>& guiTorque)
{
	boost::lock_guard<boost::mutex> lock(_guiTorqueMutex);
	_guiTorque.push_back(guiTorque);
}

double InterThread::getTimeConsumeNMS()
{
	boost::lock_guard<boost::mutex> lock(_timeConsumeNMSMutex);
	return _timeConsumeNMS;
}

void InterThread::AddToTimeConsumeNMS(const double& timeConsumeNMS)
{
	boost::lock_guard<boost::mutex> lock(_timeConsumeNMSMutex);
	_timeConsumeNMS += timeConsumeNMS;
}

double InterThread::getTimeEndNMS()
{
	boost::lock_guard<boost::mutex> lock(_timeEndNMSMutex);
	return _timeEndNMS;
}

void InterThread::AddToTimeEndNMS(const double& timeEndNMS)
{
	boost::lock_guard<boost::mutex> lock(_timeEndNMSMutex);
	_timeEndNMS += timeEndNMS;
}

double InterThread::getTimeConsumeMTU()
{
	boost::lock_guard<boost::mutex> lock(_timeConsumeMTUMutex);
	return _timeConsumeMTU;
}

void InterThread::AddToTimeConsumeMTU(const double& timeConsumeMTU)
{
	boost::lock_guard<boost::mutex> lock(_timeConsumeMTUMutex);
	_timeConsumeMTU += timeConsumeMTU;
}

int InterThread::getNbFrameNMS()
{
	boost::lock_guard<boost::mutex> lock(_nbFrameNMSMutex);
	return _nbFrameNMS;
}

void InterThread::AddToNbFrameNMS(const int& nbFrameNMS)
{
	boost::lock_guard<boost::mutex> lock(_nbFrameNMSMutex);
	_nbFrameNMS += nbFrameNMS;
}

int InterThread::getNbFrameMTU()
{
	boost::lock_guard<boost::mutex> lock(_nbFrameMTUMutex);
	return _nbFrameMTU;
}

void InterThread::AddToNbFrameMTU(const int& nbFrameMTU)
{
	boost::lock_guard<boost::mutex> lock(_nbFrameMTUMutex);
	_nbFrameMTU += nbFrameMTU;
}

int InterThread::getNbFrameEMG()
{
	boost::lock_guard<boost::mutex> lock(_nbFrameEMGMutex);
	return _nbFrameEMG;
}

void InterThread::AddToNbFrameEMG(const int& nbFrameEMG)
{
	boost::lock_guard<boost::mutex> lock(_nbFrameEMGMutex);
	_nbFrameEMG += nbFrameEMG;
}

std::vector<std::vector<double> > InterThread::getGuiMultTorqueMotor()
{
	std::vector<std::vector<double> > temp;
	boost::lock_guard<boost::mutex> lock(_guiMultTorqueMotorMutex);
	temp = _guiMultTorqueMotor;
	_guiMultTorqueMotor.clear();
	return temp;
}

void InterThread::setGuiMultTorqueMotor(const std::vector <double>& guiMultTorqueMotor)
{
	boost::lock_guard<boost::mutex> lock(_guiMultTorqueMotorMutex);
	_guiMultTorqueMotor.push_back(guiMultTorqueMotor);
}

boost::mutex InterThread::_queueLmtMutex;
boost::mutex InterThread::_queueEmgMutex;
boost::mutex InterThread::_queueAngleMutex;
boost::mutex InterThread::_queueMomentArmsMutex;
boost::mutex InterThread::_queueExternalTorqueMutex;
boost::mutex InterThread::_musclesNamesOnChannelMutex;
boost::mutex InterThread::_musclesNamesOnDofMutex;
boost::mutex InterThread::_positionMapMutex;
boost::mutex InterThread::_dofNamesMutex;
boost::mutex InterThread::_musclesNamesMutex;
boost::mutex InterThread::_libPathEMGMutex;
boost::mutex InterThread::_libPathAngleMutex;
boost::mutex InterThread::_modelFileNameMutex;
boost::mutex InterThread::_lmtTimeMutex;
boost::mutex InterThread::_EMGTimeMutex;
boost::mutex InterThread::_syncVerboseMutex;
boost::mutex InterThread::_syncGuiMutex;
boost::mutex InterThread::_endThreadMutex;
boost::mutex InterThread::_guiMuscleForceMutex;
boost::mutex InterThread::_guiTimeEmgMutex;
boost::mutex InterThread::_guiTimeLMTMutex;
boost::mutex InterThread::_guiTimePosMutex;
boost::mutex InterThread::_guiTimeTorqueMutex;
boost::mutex InterThread::_guiTorqueMutex;
boost::mutex InterThread::_guiEMGMutex;
boost::mutex InterThread::_guiIDMutex;
boost::mutex InterThread::_timeConsumeNMSMutex;
boost::mutex InterThread::_timeEndNMSMutex;
boost::mutex InterThread::_timeConsumeMTUMutex;
boost::mutex InterThread::_nbFrameNMSMutex;
boost::mutex InterThread::_nbFrameMTUMutex;
boost::mutex InterThread::_nbFrameEMGMutex;
boost::mutex InterThread::_guiMultTorqueMotorMutex;
boost::mutex InterThread::_guiWorkLoopMutex;
boost::mutex InterThread::_OFLMutex;
boost::mutex InterThread::_isometricsMaxForceMutex;
boost::barrier* InterThread::readyToStart = new boost::barrier(SyncTools::Shared::numberOfThreads);
std::list< std::vector <double> > InterThread::_queueEmg;
std::list< std::vector <double> > InterThread::_queueLmt;
std::list< std::vector <double> > InterThread::_queueAngle;
std::vector< std::list< std::vector <double> > > InterThread::_queueMomentArms;
std::list< std::vector <double> > InterThread::_queueExternalTorque;
std::map<std::string, std::vector <std::string> > InterThread::_musclesNamesOnChannel;
std::vector< std::vector < std::string > > InterThread::_musclesNamesOnDof;
std::map<std::string, double> InterThread::_positionMap;
std::vector< std::string > InterThread::_dofNames;
std::vector< std::string > InterThread::_musclesNames;
std::vector<std::vector<double> > InterThread::_guiTorque;
std::vector<std::vector<double> > InterThread::_guiEMG;
std::vector<std::vector<double> > InterThread::_guiID;
std::vector<std::vector<double> > InterThread::_guiMultTorqueMotor;
std::vector<std::vector<double> > InterThread::_guiForce;
std::vector<std::vector<double> > InterThread::_guiFiberLength;
std::vector <double> InterThread::_guiMuscleForce;
std::vector <double> InterThread::_guiTimeEmg;
std::vector <double> InterThread::_guiTimeLMT;
std::vector <double> InterThread::_guiTimePos;
std::vector <double> InterThread::_guiTimeTorque;
std::vector <double> InterThread::_guiTimeWorkLoop;
std::vector< double > InterThread::_isometricsMaxForce;
std::vector< double > InterThread::_OFL;
std::string InterThread::_libPathEMG;
std::string InterThread::_libPathAngle;
std::string InterThread::_modelFileName;
double InterThread::_lmtTime = -1;
double InterThread::_EMGTime = -1;
double InterThread::_timeConsumeNMS = 0;
double InterThread::_timeEndNMS = 0;
double InterThread::_timeConsumeMTU = 0;
int InterThread::_syncVerbose = 0;
int InterThread::_nbFrameNMS = 0;
int InterThread::_nbFrameMTU = 0;
int InterThread::_nbFrameEMG = 0;
bool InterThread::_endThread = false;
bool InterThread::_syncGui = false;
Semaphore InterThread::_queueEmgSemFull;
Semaphore InterThread::_queueEmgSemEmpty(SyncTools::Shared::queueBuffer);
Semaphore InterThread::_queueLmtSemFull;
Semaphore InterThread::_queueLmtSemEmpty(SyncTools::Shared::queueBuffer);
Semaphore InterThread::_queueAngleSemFull;
Semaphore InterThread::_queueAngleSemEmpty(SyncTools::Shared::queueBuffer);
Semaphore InterThread::_queueMomentArmsSemFull;
Semaphore InterThread::_queueMomentArmsSemEmpty(SyncTools::Shared::queueBuffer);
Semaphore InterThread::_queueExternalTorqueSemFull;
Semaphore InterThread::_queueExternalTorqueSemEmpty(SyncTools::Shared::queueBuffer);
Semaphore InterThread::_musclesNamesOnChannelSem;
Semaphore InterThread::_musclesNamesOnDofSem;
Semaphore InterThread::_dofNamesSem;
Semaphore InterThread::_musclesNamesSem;
Semaphore InterThread::_libPathEMGSem;
Semaphore InterThread::_libPathAngleSem;
Semaphore InterThread::_semModelFileName;
Semaphore InterThread::_syncVerboseSem;
Semaphore InterThread::_syncGuiSem;
Semaphore InterThread::lmtProducingDone;
Semaphore InterThread::emgProducingDone;
Semaphore InterThread::angleProducingDone;
Semaphore InterThread::consumerDone;
Semaphore InterThread::NMSconsumerDoneEMG;
Semaphore InterThread::NMSconsumerDoneAngle;
Semaphore InterThread::_isometricsMaxForceSem;
Semaphore InterThread::_OFLSem;