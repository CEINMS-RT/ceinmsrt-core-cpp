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
#include "SyncToolsCal.h"

namespace SyncToolsCal {
    namespace Shared {

    std::vector < std::vector < std::vector < double > > > torqueCal;
    boost::mutex torqueCalMutex;

    std::vector < double > vecParamCal;
	boost::mutex vecParamCalMutex;

    std::vector < std::vector < std::vector < double > > > torqueBase;
    Semaphore torqueBaseReady(0);

    std::vector < std::vector < double > > timeIKBase;
	Semaphore timeIKReady(0);
	
	std::vector < std::vector < double > > timeIDBase;
	Semaphore timeIDReady(0);

    std::vector < double > vecParamBase;
    Semaphore vecParamBaseReady(0);

    std::vector < double > vecParamUB;
	Semaphore vecParamUBReady(0);

	std::vector < double > vecParamLB;
	Semaphore vecParamLBReady(0);

	unsigned int strengthCoeff;
	Semaphore strengthCoeffReady(0);

    vector<string> dofNames;
    Semaphore dofNamesSem(0);

    vector<unsigned int> musclesIndexToCalibrate;
	Semaphore musclesIndexToCalibrateSem(0);

    vector<unsigned int> dofIndexToCalibrate;
	Semaphore dofIndexToCalibrateSem(0);

    vector<string> musclesNames;
	Semaphore musclesNamesSem(0);

    vector<string> trialNames;
	Semaphore trialNamesSem(0);

#ifdef USE_GUI
    boost::barrier readyToStart(2);
#else	
	boost::barrier readyToStart(1);
#endif

    bool endThread;
    boost::mutex endThreadMutex;
	
	bool endGui;
    boost::mutex endGuiMutex;

    double fOpt;
	boost::mutex fOptMutex;
	
	bool startNewWin;
    boost::mutex startNewWinMutex;
	
	bool stopWin;
    boost::mutex stopWinWinMutex;

    double mpiIteration;
    boost::mutex mpiIterationMutex;
	
	bool finish;
    boost::mutex finishMutex;
	
	int nbOfEval;
    boost::mutex nbOfEvalMutex;
	
	int maxEval;
	Semaphore maxEvalReady;
	
	double minConv;
    Semaphore minConvReady;
	
	double convergence;
    boost::mutex convergenceMutex;

    };
};
