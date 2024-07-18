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