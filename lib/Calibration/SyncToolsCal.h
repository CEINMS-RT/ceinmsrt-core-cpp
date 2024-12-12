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

#ifndef SyncToolsCal_h
#define SyncToolsCal_h

#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/condition_variable.hpp>

#include "Semaphore.h"

#include <string>
using std::string;
#include <list>
using std::list;
#include <vector>
using std::vector;
#include <map>
using std::map;

namespace SyncToolsCal {
    namespace Shared{

    extern std::vector < std::vector < std::vector < double > > > torqueCal;
    extern boost::mutex torqueCalMutex;

    extern std::vector < double > vecParamCal;
    extern boost::mutex vecParamCalMutex;

    extern std::vector < std::vector < std::vector < double > > > torqueBase;
    extern Semaphore torqueBaseReady;

    extern std::vector < std::vector < double > > timeIKBase;
	extern Semaphore timeIKReady;
	
	extern std::vector < std::vector < double > > timeIDBase;
	extern Semaphore timeIDReady;

    extern std::vector < double > vecParamBase;
    extern Semaphore vecParamBaseReady;

    extern std::vector < double > vecParamUB;
    extern Semaphore vecParamUBReady;

    extern std::vector < double > vecParamLB;
    extern Semaphore vecParamLBReady;

    extern vector<string> dofNames;
    extern Semaphore dofNamesSem;

    extern vector<string> musclesNames;
    extern Semaphore musclesNamesSem;

    extern vector<string> trialNames;
    extern Semaphore trialNamesSem;

    extern vector<unsigned int> musclesIndexToCalibrate;
	extern Semaphore musclesIndexToCalibrateSem;

	extern vector<unsigned int> dofIndexToCalibrate;
	extern Semaphore dofIndexToCalibrateSem;

    extern boost::barrier readyToStart;

    extern bool endThread;
    extern boost::mutex endThreadMutex;
	
	extern bool endGui;
    extern boost::mutex endGuiMutex;
	
	extern bool startNewWin;
    extern boost::mutex startNewWinMutex;
	
	extern bool stopWin;
    extern boost::mutex stopWinWinMutex;

    extern unsigned int strengthCoeff;
    extern Semaphore strengthCoeffReady;

    extern double fOpt;
    extern boost::mutex fOptMutex;

    extern double mpiIteration;
    extern boost::mutex mpiIterationMutex;
	
	extern bool finish;
    extern boost::mutex finishMutex;
	
	extern int nbOfEval;
    extern boost::mutex nbOfEvalMutex;
	
	extern int maxEval;
    extern Semaphore maxEvalReady;
	
	extern double minConv;
    extern Semaphore minConvReady;
	
	extern double convergence;
    extern boost::mutex convergenceMutex;

    };
};

#endif
