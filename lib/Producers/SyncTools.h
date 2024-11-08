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
#ifndef SyncTools_h
#define SyncTools_h

#include <CommonCEINMS.h>

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

namespace SyncTools
{
	namespace Shared
	{
		extern float globalTimeLimit;
		extern const int queueBuffer;
		extern const unsigned int numberOfThreads;

// 		extern std::list< std::vector <double> > queueEmg;
// 		extern boost::mutex queueEmgMutex;
// 		extern Semaphore queueEmgSemFull, queueEmgSemEmpty;
// 
// 		extern std::list< std::vector <double> > queueLmt;
// 		extern boost::mutex queueLmtMutex;
// 		extern Semaphore queueLmtSemFull, queueLmtSemEmpty;
// 
// 		extern std::vector< std::list< std::vector <double> > > queueMomentArms;
// 		extern boost::mutex queueMomentArmsMutex;
// 		extern Semaphore queueMomentArmsSemFull, queueMomentArmsSemEmpty;
// 
// 		extern std::vector< std::list< std::vector <double> > > queueExternalTorque;
// 		extern boost::mutex queueExternalTorqueMutex;
// 		extern Semaphore queueExternalTorqueSemFull, queueExternalTorqueSemEmpty;
// 
// 		extern list< vector <double> > queueAngle;
// 		extern boost::mutex queueAngleMutex;
// 		extern Semaphore queueAngleSemFull, queueAngleSemEmpty;
// 
// 		extern std::vector< std::vector < std::string > > musclesNamesOnDof;
// 		extern boost::mutex musclesNamesOnDofMutex;
// 
// 		extern std::vector< std::string > dofNamesWithExtTorque;
// 		extern boost::mutex dofNamesWithExtTorqueMutex;
// 
// 		extern std::vector<std::string> dofNamesWithExtAngle;
// 		extern boost::mutex dofNamesWithExtAngleMutex;
// 
// 		extern boost::barrier readyToStart;
// 
// 		extern std::map<std::string, std::vector <std::string> > musclesNamesOnChannel;
// 		extern Semaphore musclesNamesOnChannelSem;
// 
// 		extern std::vector< std::string > dofNames;
// 		extern Semaphore dofNamesSem;
// 
// 		extern std::vector< std::string > musclesNames;
// 		extern Semaphore musclesNamesSem;
// 		extern boost::mutex musclesNamesMutex;
// 
// 		extern Semaphore lmtProducingDone;
// 		extern Semaphore emgProducingDone;
// 		extern Semaphore angleProducingDone;
// 		extern Semaphore consumerDone;
// 		extern Semaphore NMSconsumerDoneEMG;
// 		extern Semaphore NMSconsumerDoneAngle;
// 
// 		extern bool endThread;
// 		extern boost::mutex endThreadMutex;
// 
// 		extern string libPathEMG;
// 		extern boost::mutex libPathEMGMutex;
// 
// 		extern string libPathAngle;
// 		extern boost::mutex libPathAngleMutex;
// 
// 		extern double lmtTime;
// 		extern boost::mutex lmtTimeMutex;
// 
// 		extern double EMGTime;
// 		extern boost::mutex EMGTimeMutex;
// 
// 		extern vector <double> muscleForceGui, timeEmg, timeTorque;
// 		extern std::vector<std::vector<double> > torqueGui, emgGui, idGui;
// 		extern double timeConsumeNMS, timeEndNMS, timeConsumeMTU;
// 		extern int nbFrameNMS, nbFrameMTU, nbFrameEMG;
// 		extern boost::mutex mutexGui;
// 
// 		extern std::string modelFileName;
// 		extern Semaphore semModelFileName;
// 
// 		extern std::map<std::string, double> positionMap;
// 		extern boost::mutex positionMapMutex;
// 
// 		extern boost::mutex syncGuiMutex;
// 		extern bool syncGui;
// 
// 		extern boost::mutex syncVerboseMutex;
// 		extern int syncVerbose;

	};
};

class InterThread
{
	public:
		
		static void 		notifyAll();
		
		static bool 		getEndThread();
		static void 		setEndThread ( const bool& end );

		static bool 		getSyncGui();
		static void 		setSyncGui ( const bool& syncGui );

		static int			getSyncVerbose();
		static void 		setSyncVerbose ( const int& syncVerbose );

		static std::string 	getLibPathEMG();
		static void 		setLibPathEMG ( const std::string& libPathEMG );

		static std::string 	getLibPathAngleMutex();
		static void 		setLibPathAngleMutex ( const std::string& libPathAngleMutex );

		static std::string 	getModelFileName();
		static void 		setModelFileName ( const std::string& modelFileName );

		static double 		getEMGTime();
		static void 		setEMGTime ( const double& emgTime );

		static double 		getLmtTime();
		static void 		setLmtTime ( const double& lmtTime );


		static std::vector<double> 	getEMG();
		static void 				setEMG ( const std::vector<double>& emg );

		static std::vector<double> 	getLMT();
		static void 				setLMT ( const std::vector<double>& lmt );
		static bool					isEmptyLMT();

		static std::vector<double> 	getOptimaFiberLength();
		static void 				setOptimaFiberLength(const std::vector<double>& OFL);

		static std::vector<double> 	getIsometricsMaxForce();
		static void 				setIsometricsMaxForce(const std::vector<double>& isometricsMaxForce);

		static std::vector<double> 	getAngle();
		static void 				setAngle ( const std::vector<double>& angle );

		static std::vector<double> 	getMomentsArm ( unsigned int whichDof );
		static void 				setMomentsArm ( const std::vector<double>& momentsArm, unsigned int whichDof );

		static std::vector<double> 	getExternalTorque (  );
		static void 				setExternalTorque ( const std::vector<double>& externalTorque );

		static std::vector< std::vector < std::string > > 			getMusclesNamesOnDof();
		static void 												setMusclesNamesOnDof ( const std::vector< std::vector < std::string > >& musclesNamesOnDof );

		static std::map<std::string, std::vector <std::string> > 	getMusclesNamesOnChannel();
		static void 												setMusclesNamesOnChannel ( const std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel );

		static std::vector< std::string > 			getMusclesNames();
		static void 								setMusclesNames ( const std::vector< std::string >& musclesNames );

		static std::vector< std::string > 			getDofNames();
		static void 								setDofNames ( const std::vector< std::string >& dofNames );

		static std::map<std::string, double>		getPositionMap();
		static void 								setPositionMap ( const std::map<std::string, double>& postionMap );

		static std::vector <double>					getGuiTimeEmg();
		static void									setGuiTimeEmg ( const double& guiTimeEmg );


		static std::vector <double>					getGuiTimeLMT();
		static void									setGuiTimeLMT(const double& guiTimeLMT);

		static std::vector <double>					getGuiTimePos();
		static void									setGuiTimePos(const double& guiTimePos);

		static std::vector <double>					getGuiMuscleForce();
		static void									setGuiMuscleForce ( const std::vector <double>& guiMuscleForce );

		static std::vector <double>					getGuiTimeTorque();
		static void									setGuiTimeTorque ( const double& guiTimeTorque );

		static std::vector<std::vector<double> >	getGuiMultTorqueMotor();
		static void									setGuiMultTorqueMotor ( const std::vector <double>& guiMultTorqueMotor );

		static void									getGuiEMG(std::vector<std::vector<double> >& guiEMG, std::vector<double>& guiTimeEmg);
		static void									setGuiEMG(const std::vector<double>& guiEMG, const double& guiTimeEmg);

		static void									getGuiWorkLoop(std::vector<std::vector<double> >& guiForce, std::vector<std::vector<double> >& guiFiberLength, std::vector<double>& guiTimeWorkLoop);
		static void									setGuiWorkLoop(const std::vector<double>& guiForce, const std::vector<double>& guiFiberLength, const double& guiTimeWorkLoop);

		static std::vector<std::vector<double> >	getGuiID();
		static void									setGuiID ( const std::vector<double>& guiID );

		static std::vector<std::vector<double> >	getGuiTorque();
		static void									setGuiTorque ( const std::vector<double>& guiTorque );

		static double 			getTimeConsumeNMS();
		static void 			AddToTimeConsumeNMS ( const double& timeConsumeNMS );

		static double 			getTimeEndNMS();
		static void 			AddToTimeEndNMS ( const double& timeEndNMS );

		static double 			getTimeConsumeMTU();
		static void 			AddToTimeConsumeMTU ( const double& timeConsumeMTU );

		static int 				getNbFrameNMS();
		static void 			AddToNbFrameNMS ( const int& nbFrameNMS );

		static int 				getNbFrameMTU();
		static void 			AddToNbFrameMTU ( const int& nbFrameMTU );

		static int 				getNbFrameEMG();
		static void 			AddToNbFrameEMG ( const int& nbFrameEMG );
		
		

		static Semaphore 		lmtProducingDone;
		static Semaphore 		emgProducingDone;
		static Semaphore 		angleProducingDone;
		static Semaphore 		consumerDone;
		static Semaphore 		NMSconsumerDoneEMG;
		static Semaphore 		NMSconsumerDoneAngle;

		static boost::barrier* 	readyToStart;

	protected:

		static std::list< std::vector <double> >	_queueEmg;
		static boost::mutex 						_queueEmgMutex;
		static Semaphore 							_queueEmgSemFull;
		static Semaphore 							_queueEmgSemEmpty;

		static std::list< std::vector <double> > 	_queueLmt;
		static boost::mutex 						_queueLmtMutex;
		static Semaphore 							_queueLmtSemFull;
		static Semaphore 							_queueLmtSemEmpty;

		static std::list< std::vector <double> > 	_queueAngle;
		static boost::mutex 						_queueAngleMutex;
		static Semaphore 							_queueAngleSemFull;
		static Semaphore 							_queueAngleSemEmpty;

		static std::vector< std::list< std::vector <double> > > _queueMomentArms;
		static boost::mutex 									_queueMomentArmsMutex;
		static Semaphore 										_queueMomentArmsSemFull;
		static Semaphore 										_queueMomentArmsSemEmpty;

		static std::list< std::vector <double> > _queueExternalTorque;
		static boost::mutex 									_queueExternalTorqueMutex;
		static Semaphore 										_queueExternalTorqueSemFull;
		static Semaphore 										_queueExternalTorqueSemEmpty;

		static std::map<std::string, std::vector <std::string> > 	_musclesNamesOnChannel;
		static boost::mutex 										_musclesNamesOnChannelMutex;
		static Semaphore 											_musclesNamesOnChannelSem;

		static std::vector< std::vector < std::string > > 			_musclesNamesOnDof;
		static boost::mutex 										_musclesNamesOnDofMutex;
		static Semaphore 											_musclesNamesOnDofSem;

		static std::map<std::string, double> 		_positionMap;
		static boost::mutex 						_positionMapMutex;

		static std::vector< std::string > 			_dofNames;
		static boost::mutex 						_dofNamesMutex;
		static Semaphore 							_dofNamesSem;

		static std::vector< double > 				_OFL;
		static boost::mutex 						_OFLMutex;
		static Semaphore 							_OFLSem;

		static std::vector< double > 				_isometricsMaxForce;
		static boost::mutex 						_isometricsMaxForceMutex;
		static Semaphore 							_isometricsMaxForceSem;

		static std::vector< std::string > 			_musclesNames;
		static boost::mutex 						_musclesNamesMutex;
		static Semaphore 							_musclesNamesSem;

		static std::string 							_libPathEMG;
		static boost::mutex 						_libPathEMGMutex;
		static Semaphore							_libPathEMGSem;

		static std::string 							_libPathAngle;
		static boost::mutex 						_libPathAngleMutex;
		static Semaphore							_libPathAngleSem;

		static std::string 							_modelFileName;
		static boost::mutex 						_modelFileNameMutex;
		static Semaphore 							_semModelFileName;

		static double 								_lmtTime;
		static boost::mutex 						_lmtTimeMutex;

		static double 								_EMGTime;
		static boost::mutex 						_EMGTimeMutex;

		static int 									_syncVerbose;
		static boost::mutex 						_syncVerboseMutex;
		static Semaphore							_syncVerboseSem;

		static bool 								_syncGui;
		static boost::mutex 						_syncGuiMutex;
		static Semaphore							_syncGuiSem;

		static bool 								_endThread;
		static boost::mutex 						_endThreadMutex;

		static std::vector <double>					_guiMuscleForce;
		static boost::mutex 						_guiMuscleForceMutex;

		static std::vector <double>					_guiTimeEmg;
		static boost::mutex 						_guiTimeEmgMutex;

		static std::vector <double>					_guiTimeLMT;
		static boost::mutex 						_guiTimeLMTMutex;

		static std::vector <double>					_guiTimePos;
		static boost::mutex 						_guiTimePosMutex;

		static std::vector <double>					_guiTimeTorque;
		static boost::mutex 						_guiTimeTorqueMutex;

		static std::vector<std::vector<double> > 	_guiTorque;
		static boost::mutex 						_guiTorqueMutex;
		
		static std::vector<std::vector<double> > 	_guiMultTorqueMotor;
		static boost::mutex 						_guiMultTorqueMotorMutex;

		static std::vector<std::vector<double> > 	_guiEMG;
		static boost::mutex 						_guiEMGMutex;

		static std::vector<std::vector<double> > 	_guiForce;
		static std::vector<std::vector<double> > 	_guiFiberLength;
		static std::vector<double> 					_guiTimeWorkLoop;
		static boost::mutex 						_guiWorkLoopMutex;

		static std::vector<std::vector<double> > 	_guiID;
		static boost::mutex 						_guiIDMutex;

		static double 								_timeConsumeNMS;
		static boost::mutex 						_timeConsumeNMSMutex;

		static double 								_timeEndNMS;
		static boost::mutex 						_timeEndNMSMutex;

		static double 								_timeConsumeMTU;
		static boost::mutex 						_timeConsumeMTUMutex;

		static int 									_nbFrameNMS;
		static boost::mutex 						_nbFrameNMSMutex;

		static int 									_nbFrameMTU;
		static boost::mutex 						_nbFrameMTUMutex;

		static int 									_nbFrameEMG;
		static boost::mutex 						_nbFrameEMGMutex;

};


#endif
