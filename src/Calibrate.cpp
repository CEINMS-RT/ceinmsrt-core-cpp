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

#include <iostream>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <vector>

#include <CommonCEINMS.h>

// to create the NMS model
#include "NMSmodel.h"
#include "SetupDataStructure.h"
#include <xercesc/util/PlatformUtils.hpp>


// to read the emg, lmt, ma, and torque files
#include "EMGDataFromFile.h"
#include "DataFromFile.h"
#include "TorquesComputation.h"
#include "TorquesComputationRT.h"
#include "ComputationMode_Default.h"
#include "ComputationMode_Fast.h"
#include "ExponentialActivation.h"
#include "Activation/ExponentialActivationRT.h"
#include "StiffTendon.h"
#include "ElasticTendon.h"
#include "ElasticTendon_BiSec.h"
#include "Curve.h"

#include "ExecutionXmlReader.h"



// to optimize
#include "SimulatedAnnealing.h"
#include "ShapeFactor.h"
#include "StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange.h"
#include "StrengthCoefficients_ShapeFactor_TendonSlackLength_single.h"
#include "StrengthCoefficients_ShapeFactor_C1_C2_TendonSlackLength_single.h"
#include "StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only.h"
#include "GroupedStrengthCoefficients_IndividualLstLom.h"
#include "SumMinObjectiveFunction_singleF.h"
#include "simulatedAnnealing.hxx"

#ifdef USE_GUI
#include <QApplication>
#include "mainwindowCal.h"
#include <QMainWindow>
#include <QSplashScreen>
#include <QIcon>
#include "CalManager.h"
#endif

#ifdef USE_OPENSIM
#include "MuscleTendonScaling.h"
//Spline
#include "MuscleAnalyseForSpline.h"
#include "MTUSplineData.h"
#include "MTUSplineDataWrite.h"
#endif

#include <thread>
#include <boost/thread.hpp>
#include "boost/program_options.hpp"
#include <boost/preprocessor/stringize.hpp>

#include "SyncToolsCal.h"
#include <csignal>
#include "ExecutionSimulatedAnnealing.h"
#include <boost/shared_ptr.hpp>

#include <tclap/CmdLine.h>

template<typename T1>
void optimiseThread ( T1& t1 );

template<typename T1, typename T2, typename T3>
void runThreadsWithGui ( T1& t1, T2& t2, T3& t3 );

template<typename T1>
void runThread ( T1& t1 );

void CalSigintHandler ( int sig );

void printHeader();

void printAuthors();

void CLIOption ( const int& argc, char** argv, std::string& exect, std::string& simulatedAnnealingXML, bool& exectFound,
		bool& gui, int& verbose, double& endTimer);

void ComputeSpline ( const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, const int& verbose );

template <typename SetupDataStruct, typename Subject>
boost::shared_ptr<SetupDataStruct> setupSubject ( Subject& mySubject, const std::string& ceinmsSubjectXml );

template<typename Subject>
void preScaling ( Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, const int& verbose );

template<typename Subject, typename TorqueComput, typename SimAnnealing>
void calibrate ( Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing,
		const std::string& ceinmsSubjectXml, const int& verbose, const std::string& simulatedAnnealingXML, const bool& gui, const double& endTimer);

template<typename Subject, typename TorqueComput, typename SimAnnealing, typename SetupDataStruct>
void PreScaleAndCalibrate ( const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, const int& verbose,
		const std::string& simulatedAnnealingXML, const bool& gui, const double& endTimer);

int argc_global;
char** argv_global;	

#ifdef USE_GUI
QApplication* a;
QSplashScreen* splash;
#endif

int main ( int argc, char** argv )
{
	std::cout << "ok" << std::endl << std::flush;

	SyncToolsCal::Shared::endThreadMutex.lock();
	SyncToolsCal::Shared::endThread = false;
	SyncToolsCal::Shared::endThreadMutex.unlock();
	
	COUT << std::endl;
	
#ifdef USE_GUI
	SyncToolsCal::Shared::endGuiMutex.lock();
	SyncToolsCal::Shared::endGui = false;
	SyncToolsCal::Shared::endGuiMutex.unlock();
#endif

	signal ( SIGINT, CalSigintHandler );

	std::string simulatedAnnealingXML = "cfg/Guillaume/simulatedAnnealing.xml";
	std::string exect = "cfg/Guillaume/executionRT.xml";
	int verbose = 4;
	bool gui = false;
	bool exectFound = false;
	double endTimer = 0.0;

	std::cout << "ok" << std::endl << std::flush;

	CLIOption ( argc, argv, exect, simulatedAnnealingXML, exectFound, gui, verbose, endTimer);

#ifdef USE_GUI
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	QApplication::setGraphicsSystem ( "raster" );
#endif
	QApplication a1 ( argc, argv );
	a = &a1;
// 	QPixmap pixmap ( "CEINMS-RT_Calibration_V1.png" );
	QPixmap pixmap ( "logo-ceinms-rt-white-v.png" );
	QSplashScreen splash1 ( pixmap );
	splash = &splash1;

	if ( gui )
	{
// 		a1.setWindowIcon ( QIcon ( "CEINMS-RT_Calibration_V2.png" ) );

		a1.setWindowIcon ( QIcon ( "logo-ceinms-rt-white-v.png" ) );
		splash1.show();
		a1.processEvents();
	}

#endif

#ifdef VERBOSE

	printHeader();
	printAuthors();

#endif

	xercesc::XMLPlatformUtils::Initialize();
	

	ExecutionSimulatedAnnealing executionSimulatedAnnealing ( simulatedAnnealingXML );

	ComputeSpline ( executionSimulatedAnnealing, verbose );

	if ( verbose > 1 )
		COUT << "Dofs To Calibrate: " << executionSimulatedAnnealing.getDOFToCalibrate().size() << std::endl;

	SyncToolsCal::Shared::trialNames = executionSimulatedAnnealing.getTrialsName();
	SyncToolsCal::Shared::trialNamesSem.notify();
	

	ExecutionXmlReader executionCfg ( exect );
	
	NMSModelCfg::RunMode runMode = executionCfg.getRunMode();
	

	switch ( runMode )
	{
		
		case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOnline:
		{
			typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
			typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
			std::cout << "RealTimeOpenLoopExponentialActivationStiffTendonOnline" << std::endl;
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer );
			}
			if (executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure >(executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor)
			{
				typedef SimulatedAnnealing < ShapeFactor<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::GroupedStrengthCoefficients_IndividualLstLom)
			{
				typedef SimulatedAnnealing < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			break;
		}

		case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOffline:
		{
			typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
			typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
			
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if (executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure >(executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor)
			{
				typedef SimulatedAnnealing < ShapeFactor<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::GroupedStrengthCoefficients_IndividualLstLom)
			{
				typedef SimulatedAnnealing < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			break;
		}

		case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline:
		{
			typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
			typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
			
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if (executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure >(executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor)
			{
				typedef SimulatedAnnealing < ShapeFactor<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::GroupedStrengthCoefficients_IndividualLstLom)
			{
				typedef SimulatedAnnealing < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			break;
		}

		case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOffline:
		{
			typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
			typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
			
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if (executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure >(executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor)
			{
				typedef SimulatedAnnealing < ShapeFactor<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::GroupedStrengthCoefficients_IndividualLstLom)
			{
				typedef SimulatedAnnealing < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			break;
		}

		case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOnline:
		{
			typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
			typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
			
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if (executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure >(executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor)
			{
				typedef SimulatedAnnealing < ShapeFactor<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::GroupedStrengthCoefficients_IndividualLstLom)
			{
				typedef SimulatedAnnealing < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			
			break;
		}

		case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOffline:
		{
			typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
			typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
			
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if (executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange)
			{
				typedef SimulatedAnnealing < StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure >(executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::ShapeFactor)
			{
				typedef SimulatedAnnealing < ShapeFactor<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			if(executionSimulatedAnnealing.GetCalibMode() == ExecutionSimulatedAnnealing::GroupedStrengthCoefficients_IndividualLstLom)
			{
				typedef SimulatedAnnealing < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				typedef SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > MySetupDataStructure;
				PreScaleAndCalibrate<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing, MySetupDataStructure > ( executionSimulatedAnnealing, verbose, simulatedAnnealingXML, gui, endTimer);
			}
			
			break;
		}
		default:
		{
			COUT << runMode <<std::endl;
			COUT << "Implementation not available yet. Verify you XML configuration file\n";
			break;
		}
	}
	
	xercesc::XMLPlatformUtils::Terminate();
	return 0;
}

template<typename T1>
void optimiseThread ( T1& t1 )
{
	t1.optimize();
}

template<typename T1, typename T2, typename T3>
void runThreadsWithGui ( T1& t1, T2& t2, T3& t3 )
{
#ifdef USE_GUI
	SyncToolsCal::Shared::endGuiMutex.lock();
	SyncToolsCal::Shared::endGui = false;
	SyncToolsCal::Shared::endGuiMutex.unlock();
#endif
	boost::thread thread1 ( optimiseThread<T1>, boost::ref ( t1 ) );
#ifdef USE_GUI
	//COUT << "gui" << std::endl;
	MainWindow gui;
// 	t3.finish ( &gui );
	gui.show();
	t2.exec();
#endif
	thread1.join();
}

template<typename T1>
void runThread ( T1& t1 )
{
	boost::thread thread1 ( optimiseThread<T1>, boost::ref ( t1 ) );
	thread1.join();
}

void CalSigintHandler ( int sig )
{
	COUT << "Quitting..." << std::endl;
	SyncToolsCal::Shared::endThreadMutex.lock();
	SyncToolsCal::Shared::endThread = true;
	SyncToolsCal::Shared::endThreadMutex.unlock();
}

void CLIOption ( const int& argc, char** argv, std::string& exect, std::string& simulatedAnnealingXML, bool& exectFound, bool& gui, int& verbose, double& endTimer)
{
	try {
		// Define the command line object, and insert a message
		// that describes the program. The "Command description message" 
		// is printed last in the help text. The second argument is the 
		// delimiter (usually space) and the last one is the version number. 
		// The CmdLine object parses the argv array based on the Arg objects
		// that it contains. 
		TCLAP::CmdLine cmd("Command description message", ' ', "0.9");

		// Define a value argument and add it to the command line.
		// A value arg defines a flag and a type of value that it expects,
		// such as "-n Bishop".
		TCLAP::ValueArg<std::string> nameArgS("s", "Simulated", "NSimulated annealing XML file for the calibration. See simulatedAnnealing.xsd in XSD directory for more information.", true, "default", "string");

		TCLAP::ValueArg<std::string> nameArgE("e", "execution", "Execution xml file option. See execution.xsd in XSD directory for more information.", true, "default", "string");

		TCLAP::ValueArg<int> nameArgV("v", "verbose", "Verbose option. arg (int) is the level of verbose output (0 no output, 1 basic output, 2 debug information and 3 in-loop debug).", false, 0, "int");

		// timer to kill calibration. initilize as 0 and takes double in minutes. If it is larger than 0, then stop calibration after an iteration that exceeds the input number of minutes.
		TCLAP::ValueArg<double> nameArgT("t", "endTimer", "Stop Calibrate after a certain period of time (seconds). 0: no timing limit, other positive double number for 'n' minutes", false, 0, "double");

		// Add the argument nameArg to the CmdLine object. The CmdLine object
		// uses this Arg to parse the command line.
		cmd.add(nameArgS);
		cmd.add(nameArgE);
		cmd.add(nameArgV);
		cmd.add(nameArgT);  // receive timer from the command line
		

		// Define a switch and add it to the command line.
		// A switch arg is a boolean argument and only defines a flag that
		// indicates true or false.  In this example the SwitchArg adds itself
		// to the CmdLine object as part of the constructor.  This eliminates
		// the need to call the cmd.add() method.  All args have support in
		// their constructors to add themselves directly to the CmdLine object.
		// It doesn't matter which idiom you choose, they accomplish the same thing.
		TCLAP::SwitchArg guiCMD("g", "gui", "Use the graphical interface", cmd, false);

		// Parse the argv array.
		cmd.parse(argc, argv);

		// Get the value parsed by each arg. 
		simulatedAnnealingXML = nameArgS.getValue();
		exect = nameArgE.getValue();
		gui = guiCMD.getValue();
		verbose = nameArgV.getValue();
		endTimer = nameArgT.getValue();
	}
	catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
	}
}

void printHeader()
{

#ifdef UNIX
	std::cout << "\033[0;33m------------------------------------------------" << std::endl;
	std::cout << " NEUROMUSCOLOKELETAL MODEL: CALIBRATION " << std::endl;
	std::cout << " Calibration using simulated annealing" << std::endl;
	std::cout << "------------------------------------------------\033[0m" << std::endl;
#endif
#ifdef WIN32
	std::cout << "------------------------------------------------" << std::endl;
	std::cout << " NEUROMUSCOLOKELETAL MODEL: CALIBRATION " << std::endl;
	std::cout << " Calibration using simulated annealing" << std::endl;
	std::cout << "------------------------------------------------" << std::endl;
#endif
}

void printAuthors()
{
	time_t now = time(0);
	tm* gmtm = gmtime(&now);
#ifdef UNIX
	cout << "\033[3;34mCopyright (C) " << gmtm->tm_year + 1900 <<std::endl;
	cout << "David LLoyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato, Guillaume Durandau\033[0m\n\n";
#endif
#ifdef WIN32
	cout << "Copyright (C) " << gmtm->tm_year + 1900 <<std::endl; 
	cout << "David LLoyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato, Guillaume Durandau\n";
#endif
}

void ComputeSpline ( const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, const int& verbose )
{
#ifdef USE_OPENSIM

	if ( executionSimulatedAnnealing.getUseSpline() )
	{
		if ( verbose > 0 )
			COUT << "\033[0;32mComputing the Spline for " << executionSimulatedAnnealing.getNameOfSubject() << ".\033[0m" << std::endl;

		MuscleAnalyseForSpline muscleAnalyse ( executionSimulatedAnnealing.getSubjectXML(),
				executionSimulatedAnnealing.getOsimFileSpline(), executionSimulatedAnnealing.getTranslateFileSpline(),
				executionSimulatedAnnealing.getNumberOfNode(), executionSimulatedAnnealing.getPrintOption() );

		muscleAnalyse.computeAnglesStorage();

		muscleAnalyse.run ( false );

		if ( executionSimulatedAnnealing.getPrintOption() == 1 || executionSimulatedAnnealing.getPrintOption() == 3 )
			muscleAnalyse.run ( true );

		muscleAnalyse.writeLmt();
		muscleAnalyse.writeMa();

		MTUSplineDataWrite splineData ( muscleAnalyse, executionSimulatedAnnealing.getNameOfSubject() );
		splineData.computeTaskCoeffients();

		splineData.writeTaskCoefficients();
	}

#endif
}

template <typename SetupDataStruct, typename Subject>
boost::shared_ptr<SetupDataStruct> setupSubject ( Subject& mySubject, const std::string& ceinmsSubjectXml )
{
	boost::shared_ptr<SetupDataStruct> setupData;
	try{
		setupData = boost::make_shared<SetupDataStruct>(ceinmsSubjectXml);
	}catch(const std::exception& e){
		std::ostringstream oss;	
		oss << "Runtime error at "  << BOOST_PP_STRINGIZE(__FILE__) << " line " << BOOST_PP_STRINGIZE(__LINE__) << ": " << "Subject file not parsed correctly" << std::endl;
		std::cerr << oss.str() << e.what();
		// throw std::runtime_error(oss.str());
	}
	setupData->createCurves();
	setupData->createMuscles ( mySubject );
	setupData->createDoFs ( mySubject );
	setupData->createMusclesNamesOnChannel ( mySubject );
	return setupData;
}

template<typename Subject>
void preScaling ( Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, const int& verbose )
{
#ifdef USE_OPENSIM

	if ( executionSimulatedAnnealing.getUsePreScaling() )
	{
		if ( verbose > 0 )
			COUT << "\033[0;32mPre-scaling tendonSlackLength and optimalFiberLength.\033[0m" << std::endl;

		MuscleTendonScaling* mtuScaling = new  MuscleTendonScaling ( executionSimulatedAnnealing.getUnscaledOsimFilePreScaling(), executionSimulatedAnnealing.getOsimFilePreScaling(), executionSimulatedAnnealing.getSubjectXML(), executionSimulatedAnnealing.getTranslateFilePreScaling() );
		mtuScaling->setVerbose ( verbose );
		mtuScaling->run();

		std::vector<std::string> muscleNames;
		mySubject.getMuscleNames ( muscleNames );

		std::vector<double> currentTendonSlackLengths;
		mySubject.getTendonSlackLengths ( currentTendonSlackLengths );

		vector<double> currentOptimalFiberLengths;
		mySubject.getOptimalFiberLengths ( currentOptimalFiberLengths );

		for ( std::vector<std::string>::const_iterator itMuscleName = muscleNames.begin(); itMuscleName != muscleNames.end(); itMuscleName++ )
		{
			double tendonSlackLength = mtuScaling->getTendonSlackLength ( *itMuscleName );
			double optimalFiberLength = mtuScaling->getOptimalFiberLength ( *itMuscleName );


			if ( tendonSlackLength > -1 && optimalFiberLength > -1 )
			{
				const int& cpt = std::distance<std::vector<std::string>::const_iterator> ( muscleNames.begin(), itMuscleName );
				currentTendonSlackLengths.at ( cpt ) = tendonSlackLength;
				currentOptimalFiberLengths.at ( cpt ) = optimalFiberLength;
			}
		}

		mySubject.setTendonSlackLengths ( currentTendonSlackLengths );

		mySubject.setOptimalFiberLengths ( currentOptimalFiberLengths );

		delete mtuScaling; // memory leak + bug
	}

#endif
}

template<typename Subject, typename TorqueComput, typename SimAnnealing>
void calibrate ( Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing,
		const std::string& ceinmsSubjectXml, const int& verbose, const std::string& simulatedAnnealingXML, const bool& gui, const double& endTimer)
{


	if ( verbose > 0 )
		COUT << "\033[0;32mBegin Calibration.\033[0m" << std::endl;

	const std::vector<std::vector<std::string> >& dofSequenceToCalibrate = executionSimulatedAnnealing.getDOFSequenceToCalibrate();

	for ( std::vector<std::vector<std::string> >::const_iterator it = dofSequenceToCalibrate.begin(); it != dofSequenceToCalibrate.end(); it++ )
	{

		TorqueComput torquesComputation ( mySubject,
				executionSimulatedAnnealing.getTrialsDirectory(),
				executionSimulatedAnnealing.getTrialsName(),
				*it,
				executionSimulatedAnnealing.getEMGProccesing(), ceinmsSubjectXml,
				executionSimulatedAnnealing.getNameOfSubject(),
				executionSimulatedAnnealing.getTranslateFileSpline(),
				executionSimulatedAnnealing.getTrialscropMin(),
				executionSimulatedAnnealing.getTrialscropMax(),
				executionSimulatedAnnealing.getFilterEMG(), executionSimulatedAnnealing.getEMD());

		SimAnnealing annealing ( mySubject, *it, executionSimulatedAnnealing, torquesComputation );

#ifdef USE_GUI

		if ( gui )
		{
			SyncToolsCal::Shared::startNewWinMutex.lock();
			SyncToolsCal::Shared::startNewWin = true;
			SyncToolsCal::Shared::startNewWinMutex.unlock();
		}

#endif
		annealing.setGui ( gui );
		annealing.setVerbose ( verbose );
		annealing.setEndTimer(endTimer);
		annealing.optimize();

#ifdef USE_GUI

		if ( gui )
		{
			SyncToolsCal::Shared::startNewWinMutex.lock();
			SyncToolsCal::Shared::stopWin = true;
			SyncToolsCal::Shared::startNewWinMutex.unlock();
		}

#endif
	}
}

template<typename Subject, typename TorqueComput, typename SimAnnealing, typename SetupDataStruct>
void PreScaleAndCalibrate ( const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, const int& verbose, const std::string& simulatedAnnealingXML, const bool& gui, const double& endTimer)
{

	boost::timer::auto_cpu_timer auto_t;
	std::string ceinmsSubjectXmlCalibrated = executionSimulatedAnnealing.getTrialsDirectory() + "/" + executionSimulatedAnnealing.getOutputSubjectXMLName();
	std::string ceinmsSubjectXmlPreScaled = executionSimulatedAnnealing.getTrialsDirectory() + "/" + executionSimulatedAnnealing.getInputSubjectXMLName();
	Subject mySubject;
	boost::shared_ptr<SetupDataStruct> setupData;
	
	//COUT << executionSimulatedAnnealing.getSubjectXML() << std::endl;

	if ( executionSimulatedAnnealing.getUsePreScaling() )
	{
		setupData = setupSubject<SetupDataStruct, Subject> ( mySubject, executionSimulatedAnnealing.getSubjectXML() );
		preScaling<Subject> ( mySubject, executionSimulatedAnnealing, verbose );
		setupData->writeXMLCalibratedFile ( mySubject, ceinmsSubjectXmlPreScaled );
	}
	else if(executionSimulatedAnnealing.getUseCalibration())
		setupData = setupSubject<SetupDataStruct, Subject>(mySubject, ceinmsSubjectXmlPreScaled); // use model given in the optimization XML
	//	setupData = setupSubject<SetupDataStruct, Subject>(mySubject, ceinmsSubjectXmlPreScaled); // Use subjectMTUCalibrated.xml

	if ( executionSimulatedAnnealing.getUseCalibration() )
	{
		boost::thread thread1 ( calibrate<Subject, TorqueComput, SimAnnealing>, boost::ref ( mySubject ),  boost::ref ( executionSimulatedAnnealing ), boost::ref ( ceinmsSubjectXmlPreScaled ), boost::ref ( verbose ), boost::ref ( simulatedAnnealingXML ), boost::ref ( gui ), boost::ref(endTimer)); //
#ifdef USE_GUI

		if ( gui )
		{
			CalManager guiWin;
			guiWin.setSimAnnExec(executionSimulatedAnnealing);
			splash->finish ( &guiWin );
			a->exec();
		}

#endif
		thread1.join();
		COUT << "\033[0;32mEnd of Calibration.\033[0m" << std::endl;
		setupData->writeXMLCalibratedFile ( mySubject, ceinmsSubjectXmlCalibrated );

	}
}
