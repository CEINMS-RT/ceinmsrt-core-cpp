// This is part of
// NeuroMuscoloSkeletal Model Software (NMS)
// Copyright (C) 2010 David Lloyd Massimo Sartori Monica Reggiani
//
// ?? Licenza ??
//
// The authors may be contacted via:
// email: massimo.sartori@gmail.com monica.reggiani@gmail.com

#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <stdlib.h>
#include <fstream>
using std::ofstream;
using std::ifstream;

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

// to optimize
#include "SimulatedAnnealing.h"
#include "SimulatedAnnealingSpan.h"
#include "StrengthCoefficients_ShapeFactor_TendonSlackLength_single.h"
#include "StrengthCoefficients_ShapeFactor_C1_C2_TendonSlackLength_single.h"
#include "StrengthCoefficients_Single_ShapeFactor_C1_C2_TendonSlackLength_use_last_Dof_only.h"
#include "GroupedStrengthCoefficients_IndividualLstLom.h"
#include "SumMinObjectiveFunction_singleF.h"
#include "simulatedAnnealing.hxx"
#include <csignal>
#include <boost/thread.hpp>
#include "boost/program_options.hpp"

#ifdef USE_GUI
#include "mainwindowCal.h"
#endif

#include "MTU.h"

#ifdef USE_GUI
#if QT_VERSION > QT_VERSION_CHECK(5, 0, 0)
#undef qInfo
#endif
#endif

#ifdef USE_OPENSIM
#include "MuscleTendonScaling.h"
#include "MuscleAnalyseForSpline.h"
#include "MTUSplineData.h"
#include "MTUSplineDataWrite.h"
#endif

#include "ExecutionSimulatedAnnealing.h"
#include "ExecutionXmlReader.h"
#include <boost/thread/mutex.hpp>

// typedef NMSmodel<ExponentialActivationRT, StiffTendon, CurveMode::Online> NMSmodel_stiff;
// 
// typedef TorquesComputationRT < ComputationMode_Fast<NMSmodel_stiff>,
// 				NMSmodel_stiff > TorqueComputation_stiff;
// 
// typedef SimulatedAnnealingSpan <
// GroupedStrengthCoefficients_IndividualLstLom<NMSmodel_stiff>,
// 																						 SumMinObjectiveFunction_singleF<TorqueComputation_stiff>,
// 																						 TorqueComputation_stiff, NMSmodel_stiff > Siman_stiff;
// 
// //typedef SimulatedAnnealing<
// //		StrengthCoefficients_ShapeFactor_TendonSlackLength_single<NMSmodel_stiff>,
// //		SumMinObjectiveFunction_singleF<TorqueComputation_stiff>,
// //		TorqueComputation_stiff, NMSmodel_stiff> Siman_stiff;
// 
// typedef SetupDataStructure<NMSmodel_stiff> SetupDataStructure_stiff;
// 
// typedef NMSmodel_stiff MyNMSmodelType;
// typedef TorqueComputation_stiff MyTorqueComputationType;
// typedef Siman_stiff MySimanType;
// typedef SetupDataStructure_stiff MySetupDataStructureType;
// 
// string emgProccesingFile;
// string translateFileName;
// string subjectName;
// string configurationFile;
// string inputDataDirectory;
// string unscaledOsimModelName;
// string scaledOsimModelName;


string emgProccesingFile;
string translateFileName;
string subjectName;
string inputDataDirectory;
string XMLfile;
int verbose;
bool gui;

template<typename Subject, typename TorqueComput, typename SetupDat>
void threadMPI ( Subject* mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, SetupDat setupData );

void startGuiAndStartCal ( int& argc, char** argv, int ncpu );

template<typename Subject>
void preScaling ( Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, int verbose );

void CalSigintHandler ( int sig );

template<typename Subject, typename TorqueComput, typename SimAnnealing>
void calibrateMPI ( int& argc, char** argv, Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, int verbose, std::string simulatedAnnealingFile, bool gui );


template <typename T, typename CurveM>
boost::shared_ptr<SetupDataStructure<T, CurveM> > setupSubject ( T& mySubject, string configurationFile );

int main ( int argc, char** argv )
{

		int provided;
	MPI_Init_thread ( &argc, &argv, MPI_THREAD_SERIALIZED, &provided );

	if ( provided != MPI_THREAD_SERIALIZED )
	{
		COUT << "error: MPI_THREAD_SERIALIZED not accepted" << std::endl;
//		exit(0);
	}

	int ncpu, RANK, thiscpu;

	SyncToolsCal::Shared::endThreadMutex.lock();
	SyncToolsCal::Shared::endThread = false;
	SyncToolsCal::Shared::endThreadMutex.unlock();

	signal ( SIGINT, CalSigintHandler );

	MPI_Comm_size ( MPI_COMM_WORLD, &ncpu );
	MPI_Comm_rank ( MPI_COMM_WORLD, &RANK );

	thiscpu = RANK + 1;

	if ( RANK == 0 )
	{
		cout << "\033[0;33m------------------------------------------------\n";
		cout << " NEUROMUSCOLOKELETAL MODEL: CALIBRATION \n";
		cout << " Calibration using simulated annealing\n";
		cout << "------------------------------------------------\033[0m\n";
		std::cout << std::flush;
	}

	verbose = 1;
	gui = false;

	std::string simulatedAnnealingFile;
	int verbose = 1;
// 	bool gui = false;
	std::string exect;
	bool exectFound = false;

	boost::program_options::options_description desc ( "Options" );
	desc.add_options()
	( "help,h,H", "Output a small usage guide and exit successfully. No other output is generated." )
	( "Simulated,s,S", boost::program_options::value<std::string> ( &simulatedAnnealingFile ), "Simulated annealing XML file for the calibration. See simulatedAnnealing.xsd in XSD directory for more information." )
	( "verbode,v,V", boost::program_options::value<int> ( &verbose )->default_value ( 1 ), "Verbose option. arg (int) is the level of verbose output (0 no output, 1 basic output, 2 debug information and 3 in-loop debug)." )
	( "execution,e,E", boost::program_options::value<std::string> ( &exect ), "Execution xml file option. See execution.xsd in XSD directory for more information." )
	( "gui,g,G", "Use the graphical interface" );

	boost::program_options::variables_map vm;
	boost::program_options::store ( boost::program_options::parse_command_line ( argc, argv, desc ), vm );
	boost::program_options::notify ( vm );

	if ( vm.count ( "help" ) )
	{
		std::cout << desc << std::endl;;
		return 0;
	}

	if ( !vm.count ( "Simulated" ) )
	{
		std::cout << "Need -s option." << std::endl;
		std::cout << desc << std::endl;;
		return 0;
	}

	if ( !vm.count ( "execution" ) )
	{
		std::cout << "Need -e option." << std::endl;
		std::cout << desc << std::endl;;
		return 0;
	}

	if ( vm.count ( "gui" ) )
		gui = true;


	const unsigned int NumbersMaxOfIteration = 0;

	xercesc::XMLPlatformUtils::Initialize();

	ExecutionSimulatedAnnealing executionSimulatedAnnealing ( simulatedAnnealingFile );

	emgProccesingFile = executionSimulatedAnnealing.getEMGProccesing();
	translateFileName = executionSimulatedAnnealing.getTranslateFileSpline();
	subjectName = executionSimulatedAnnealing.getNameOfSubject();
	inputDataDirectory = executionSimulatedAnnealing.getTrialsDirectory();

	ExecutionXmlReader executionCfg ( exect );
	NMSModelCfg::RunMode runMode = executionCfg.getRunMode();

	SyncToolsCal::Shared::trialNames = executionSimulatedAnnealing.getTrialsName();
	SyncToolsCal::Shared::trialNamesSem.notify();

	if ( RANK == 0 )
	{

#ifdef USE_OPENSIM

		if ( executionSimulatedAnnealing.getUseSpline() )
		{
			if ( verbose > 0 )
				COUT << "\033[0;32mComputing the Spline for " << executionSimulatedAnnealing.getNameOfSubject() << ".\033[0m" << std::endl;

			MuscleAnalyseForSpline muscleAnalyse ( executionSimulatedAnnealing.getSubjectXML(), executionSimulatedAnnealing.getOsimFileSpline(), translateFileName, executionSimulatedAnnealing.getNumberOfNode(),
					executionSimulatedAnnealing.getPrintOption() );

			muscleAnalyse.computeAnglesStorage();

			muscleAnalyse.run ( false );

			if ( executionSimulatedAnnealing.getPrintOption() == 1 || executionSimulatedAnnealing.getPrintOption() == 3 )
				muscleAnalyse.run ( true );

			muscleAnalyse.writeLmt();
			muscleAnalyse.writeMa();

			MTUSplineDataWrite splineData ( muscleAnalyse, subjectName );
			splineData.computeTaskCoeffients();

			splineData.writeTaskCoefficients();
		}

#endif

		XMLfile = inputDataDirectory + "/subjectMTUCalibrated.xml";

		boost::thread* threadMaster;

		switch ( runMode )
		{
			case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOnline:
			{
				typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				MyNMSmodel* mySubject = new MyNMSmodel; // delete in the thread
				typedef boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > > MySetupData;
				std::string ceinmsSubjectXML;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
					ceinmsSubjectXML = executionSimulatedAnnealing.getSubjectXML();
				else
					ceinmsSubjectXML = XMLfile;

				 MySetupData setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Online> > ( *mySubject, ceinmsSubjectXML );
				COUT << ceinmsSubjectXML << std::endl;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
				{
					preScaling<MyNMSmodel> ( *mySubject, executionSimulatedAnnealing, verbose );
					setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				}

				threadMaster = new boost::thread ( boost::bind ( threadMPI<MyNMSmodel, MyTorqueComputation, MySetupData >,
						boost::ref ( mySubject ), boost::ref ( executionSimulatedAnnealing ), boost::ref ( setupData ) ) );
				startGuiAndStartCal ( argc, argv, ncpu );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOffline:
			{
				typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				MyNMSmodel* mySubject = new MyNMSmodel; // delete in the thread
				typedef boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > > MySetupData;
				std::string ceinmsSubjectXML;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
					ceinmsSubjectXML = executionSimulatedAnnealing.getSubjectXML();
				else
					ceinmsSubjectXML = XMLfile;

				MySetupData setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Offline> > ( *mySubject, ceinmsSubjectXML );

				if ( executionSimulatedAnnealing.getUsePreScaling() )
				{
					preScaling<MyNMSmodel> ( *mySubject, executionSimulatedAnnealing, verbose );
					setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				}

				threadMaster = new boost::thread ( boost::bind ( threadMPI<MyNMSmodel, MyTorqueComputation, MySetupData >,
						boost::ref ( mySubject ), boost::ref ( executionSimulatedAnnealing ), boost::ref ( setupData ) ) );
				startGuiAndStartCal ( argc, argv, ncpu );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				MyNMSmodel* mySubject = new MyNMSmodel; // delete in the thread
				typedef boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > > MySetupData;
				std::string ceinmsSubjectXML;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
					ceinmsSubjectXML = executionSimulatedAnnealing.getSubjectXML();
				else
					ceinmsSubjectXML = XMLfile;

				MySetupData setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Online> > ( *mySubject, ceinmsSubjectXML );

				if ( executionSimulatedAnnealing.getUsePreScaling() )
				{
					preScaling<MyNMSmodel> ( *mySubject, executionSimulatedAnnealing, verbose );
					setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				}

				threadMaster = new boost::thread ( boost::bind ( threadMPI<MyNMSmodel, MyTorqueComputation, MySetupData >,
						boost::ref ( mySubject ), boost::ref ( executionSimulatedAnnealing ), boost::ref ( setupData ) ) );
				startGuiAndStartCal ( argc, argv, ncpu );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOffline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				MyNMSmodel* mySubject = new MyNMSmodel; // delete in the thread
				typedef boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > > MySetupData;
				std::string ceinmsSubjectXML;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
					ceinmsSubjectXML = executionSimulatedAnnealing.getSubjectXML();
				else
					ceinmsSubjectXML = XMLfile;

				MySetupData setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Offline> > ( *mySubject, ceinmsSubjectXML );

				if ( executionSimulatedAnnealing.getUsePreScaling() )
				{
					preScaling<MyNMSmodel> ( *mySubject, executionSimulatedAnnealing, verbose );
					setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				}

				threadMaster = new boost::thread ( boost::bind ( threadMPI<MyNMSmodel, MyTorqueComputation, MySetupData >,
						boost::ref ( mySubject ), boost::ref ( executionSimulatedAnnealing ), boost::ref ( setupData ) ) );
				startGuiAndStartCal ( argc, argv, ncpu );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOnline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				MyNMSmodel* mySubject = new MyNMSmodel; // delete in the thread
				typedef boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > > MySetupData;
				std::string ceinmsSubjectXML;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
					ceinmsSubjectXML = executionSimulatedAnnealing.getSubjectXML();
				else
					ceinmsSubjectXML = XMLfile;

				MySetupData setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Online> > ( *mySubject, ceinmsSubjectXML );

				if ( executionSimulatedAnnealing.getUsePreScaling() )
				{
					preScaling<MyNMSmodel> ( *mySubject, executionSimulatedAnnealing, verbose );
					setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				}

				threadMaster = new boost::thread ( boost::bind ( threadMPI<MyNMSmodel, MyTorqueComputation, MySetupData >,
						boost::ref ( mySubject ), boost::ref ( executionSimulatedAnnealing ), boost::ref ( setupData ) ) );
				startGuiAndStartCal ( argc, argv, ncpu );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOffline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				MyNMSmodel* mySubject = new MyNMSmodel; // delete in the thread
				typedef boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > > MySetupData;
				std::string ceinmsSubjectXML;

				if ( executionSimulatedAnnealing.getUsePreScaling() )
					ceinmsSubjectXML = executionSimulatedAnnealing.getSubjectXML();
				else
					ceinmsSubjectXML = XMLfile;

				MySetupData setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Offline> > ( *mySubject, ceinmsSubjectXML );

				if ( executionSimulatedAnnealing.getUsePreScaling() )
				{
					preScaling<MyNMSmodel> ( *mySubject, executionSimulatedAnnealing, verbose );
					setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				}

				threadMaster = new boost::thread ( boost::bind ( threadMPI<MyNMSmodel, MyTorqueComputation, MySetupData >,
						boost::ref ( mySubject ), boost::ref ( executionSimulatedAnnealing ), boost::ref ( setupData ) ) );
				startGuiAndStartCal ( argc, argv, ncpu );
				break;
			}

			default:
			{
				COUT << runMode << endl;
				COUT << "Implementation not available yet. Verify you XML configuration file\n";
				break;
			}
		}

		threadMaster->join();
		delete threadMaster;

	}
	else
	{
		MPI_Status status;
		bool val;
		MPI_Recv ( &val, 1, MPI::BOOL, 0, MPI::ANY_TAG, MPI_COMM_WORLD, &status );

		XMLfile = inputDataDirectory + "/subjectMTUCalibrated.xml";
		std::string XMLfileFinal = inputDataDirectory + "/subjectCalibrated.xml";
		
		switch ( runMode )
		{
			case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOnline:
			{
				typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				typedef SimulatedAnnealingSpan< GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				MyNMSmodel mySubject;
				boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > > setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Online> > ( mySubject, XMLfile );
				calibrateMPI<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing> ( argc, argv, mySubject, executionSimulatedAnnealing, verbose, simulatedAnnealingFile, gui );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOffline:
			{
				typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				typedef SimulatedAnnealingSpan < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				MyNMSmodel mySubject;
				boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > > setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Offline> > ( mySubject, XMLfile );
				calibrateMPI<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing> ( argc, argv, mySubject, executionSimulatedAnnealing, verbose, simulatedAnnealingFile, gui );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				typedef SimulatedAnnealingSpan < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				MyNMSmodel mySubject;
				boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > > setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Online> > ( mySubject, XMLfile );
				calibrateMPI<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing> ( argc, argv, mySubject, executionSimulatedAnnealing, verbose, simulatedAnnealingFile, gui );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOffline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				typedef SimulatedAnnealingSpan < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				MyNMSmodel mySubject;
				boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > > setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Offline> > ( mySubject, XMLfile );
				calibrateMPI<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing> ( argc, argv, mySubject, executionSimulatedAnnealing, verbose, simulatedAnnealingFile, gui );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOnline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				typedef SimulatedAnnealingSpan < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				MyNMSmodel mySubject;
				boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Online> > > setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Online> > ( mySubject, XMLfile );
				calibrateMPI<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing> ( argc, argv, mySubject, executionSimulatedAnnealing, verbose, simulatedAnnealingFile, gui );
				break;
			}

			case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOffline:
			{
				typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
				typedef TorquesComputationRT < ComputationMode_Fast<MyNMSmodel>, MyNMSmodel > MyTorqueComputation;
				typedef SimulatedAnnealingSpan < GroupedStrengthCoefficients_IndividualLstLom<MyNMSmodel>, SumMinObjectiveFunction_singleF<MyTorqueComputation>, MyTorqueComputation, MyNMSmodel > MySimulatedAnnealing;
				MyNMSmodel mySubject;
				boost::shared_ptr<SetupDataStructure<MyNMSmodel, Curve<CurveMode::Offline> > > setupData = setupSubject<MyNMSmodel, Curve<CurveMode::Offline> > ( mySubject, XMLfile );
				calibrateMPI<MyNMSmodel, MyTorqueComputation, MySimulatedAnnealing> ( argc, argv, mySubject, executionSimulatedAnnealing, verbose, simulatedAnnealingFile, gui );
				break;
			}

			default:
			{
				COUT << runMode << endl;
				COUT << "Implementation not available yet. Verify you XML configuration file\n";
				break;
			}
		}
	}


	MPI_Finalize();

	xercesc::XMLPlatformUtils::Terminate();

	exit ( EXIT_SUCCESS );

	/*
	if ( RANK == 0 )
	{
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
		QApplication::setGraphicsSystem ( "raster" );
#endif
		QApplication a ( argc, argv );

		boost::thread thread1 ( boost::bind ( threadMPI, boost::ref ( setupData ),
				boost::ref ( mySubject ), boost::ref ( dofsToCalibrate ),
				boost::ref ( idTrials ), NumbersMaxOfIteration, boost::ref ( cropMin ), boost::ref ( cropMax ) ) );

		MainWindow gui;
		gui.show();
		a.exec();
		thread1.join();
	}*/
// 	else
// 	{
// 		MyTorqueComputationType torquesComputation ( mySubject,
// 				inputDataDirectory, idTrials, dofsToCalibrate,
// 				emgProccesingFile, configurationFile, subjectName,
// 				translateFileName, cropMin, cropMax );
// 
// 		MySimanType annealingSpan1 ( mySubject, dofsToCalibrate, confFile,
// 				torquesComputation );
// 
// 		annealingSpan1.optimizeSpan ( argc, argv );
// 	}
// 
// 	MPI_Finalize();
// 
// 	xercesc::XMLPlatformUtils::Terminate();
// 
// 	exit ( EXIT_SUCCESS );

}

void startGuiAndStartCal ( int& argc, char** argv, int ncpu )
{
	bool val = true;

	for ( int i = 0; i < ncpu; i++ )
		MPI_Send ( &val, 1, MPI::BOOL, i, 0, MPI_COMM_WORLD );

#ifdef USE_GUI

	if ( gui )
	{
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
		QApplication::setGraphicsSystem ( "raster" );
#endif
		QApplication a ( argc, argv );

		MainWindow gui;
		gui.show();
		a.exec();
	}

#endif
}

template<typename Subject>
void preScaling ( Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, int verbose )
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
void calibrateMPI ( int& argc, char** argv, Subject& mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing,
					int verbose, std::string simulatedAnnealingFile, bool gui )
{
	if ( verbose > 0 )
		COUT << "\033[0;32mBegin Calibration.\033[0m" << std::endl;

	TorqueComput torquesComputation ( mySubject,
			executionSimulatedAnnealing.getTrialsDirectory(),
			executionSimulatedAnnealing.getTrialsName(),
			executionSimulatedAnnealing.getDOFToCalibrate(),
			executionSimulatedAnnealing.getEMGProccesing(), XMLfile,
			executionSimulatedAnnealing.getNameOfSubject(),
			executionSimulatedAnnealing.getTranslateFileSpline(),
			executionSimulatedAnnealing.getTrialscropMin(),
			executionSimulatedAnnealing.getTrialscropMax(), 
			executionSimulatedAnnealing.getFilterEMG() );

	SimAnnealing annealing ( mySubject, executionSimulatedAnnealing.getDOFToCalibrate(), simulatedAnnealingFile,
			torquesComputation );

	annealing.setGui ( gui );
	annealing.setVerbose ( verbose );

	annealing.optimizeSpan ( argc, argv );
}

void CalSigintHandler ( int sig )
{
	COUT << "Quitting..." << std::endl;
	SyncToolsCal::Shared::endThreadMutex.lock();
	SyncToolsCal::Shared::endThread = true;
	SyncToolsCal::Shared::endThreadMutex.unlock();
}


template<typename Subject, typename TorqueComput, typename SetupDat>
void threadMPI ( Subject* mySubject, const ExecutionSimulatedAnnealing& executionSimulatedAnnealing, SetupDat setupData )
{

	GroupedStrengthCoefficients_IndividualLstLom<Subject> parameter (
		*mySubject, executionSimulatedAnnealing.getDOFToCalibrate() );

	TorqueComput torquesComputation ( *mySubject, 
			executionSimulatedAnnealing.getTrialsDirectory(),
			executionSimulatedAnnealing.getTrialsName(),
			executionSimulatedAnnealing.getDOFToCalibrate(),
			executionSimulatedAnnealing.getEMGProccesing(), XMLfile,
			executionSimulatedAnnealing.getNameOfSubject(),
			executionSimulatedAnnealing.getTranslateFileSpline(),
			executionSimulatedAnnealing.getTrialscropMin(),
			executionSimulatedAnnealing.getTrialscropMax(),
			executionSimulatedAnnealing.getFilterEMG() );

	int ncpu, RANK, thiscpu;

	MPI_Comm_size ( MPI_COMM_WORLD, &ncpu );
	MPI_Comm_rank ( MPI_COMM_WORLD, &RANK );

	torquesComputation.setTimeTorques ( SyncToolsCal::Shared::timeIKBase );
	torquesComputation.getInverseTorquesTimeStep ( SyncToolsCal::Shared::timeIDBase );
	torquesComputation.setInverseTorques ( SyncToolsCal::Shared::torqueBase );
	SyncToolsCal::Shared::torqueBaseReady.notify();
	SyncToolsCal::Shared::timeIDReady.notify();
	SyncToolsCal::Shared::timeIKReady.notify();
	parameter.setUpperLowerBounds ( SyncToolsCal::Shared::vecParamUB,
			SyncToolsCal::Shared::vecParamLB );
	SyncToolsCal::Shared::vecParamLBReady.notify();
	SyncToolsCal::Shared::vecParamUBReady.notify();
	SyncToolsCal::Shared::vecParamBase.resize (
		SyncToolsCal::Shared::vecParamUB.size() );
	parameter.getStartingVectorParameters ( SyncToolsCal::Shared::vecParamBase );
	SyncToolsCal::Shared::vecParamBaseReady.notify();
	SyncToolsCal::Shared::dofNames = executionSimulatedAnnealing.getDOFToCalibrate();
	mySubject->getMuscleNames ( SyncToolsCal::Shared::musclesNames );
	SyncToolsCal::Shared::dofNamesSem.notify();
	SyncToolsCal::Shared::musclesNamesSem.notify();
	mySubject->getMusclesIndexFromDofs ( SyncToolsCal::Shared::musclesIndexToCalibrate, executionSimulatedAnnealing.getDOFToCalibrate() );
	SyncToolsCal::Shared::musclesIndexToCalibrateSem.notify();
	torquesComputation.getDofsToCalibrateIndexList ( SyncToolsCal::Shared::dofIndexToCalibrate );
	SyncToolsCal::Shared::dofIndexToCalibrateSem.notify();

	MPI_Status status;
	int nbParameters = parameter.getNoParameters();
	std::vector<double> x ( nbParameters );
	std::vector<double> bestX ( nbParameters );
	std::vector<std::vector<std::vector<double> > > torques;
	std::vector<std::vector<double> > penalties;
	torquesComputation.resizeTorquesVector ( torques );
	torquesComputation.resizePenaltiesVector ( penalties );
	double bestFOpt;
	bool firstPass = true;
	unsigned int nbIteration = ncpu - 1;
	unsigned int endProcess = 0;

// 	SyncToolsCal::Shared::readyToStart.wait();

while ( true )
	{

		SyncToolsCal::Shared::endThreadMutex.lock();

		if ( SyncToolsCal::Shared::endThread )
		{
			SyncToolsCal::Shared::endThreadMutex.unlock();
			std::cout << "Quitting..." << std::endl;
			exit ( 0 );
		}

		SyncToolsCal::Shared::endThreadMutex.unlock();

		// receive message param value from any source
		MPI_Recv ( x.data(), nbParameters, MPI::DOUBLE, MPI_ANY_SOURCE, PARAM,
				MPI_COMM_WORLD, &status );

		// receive message f
		double fOpt;
		MPI_Recv ( &fOpt, 1, MPI::DOUBLE, status.MPI_SOURCE, FOPT, MPI_COMM_WORLD,
				&status );

		if ( firstPass )
		{
			bestFOpt = fOpt;
			bestX = x;
			SyncToolsCal::Shared::fOptMutex.lock();
			SyncToolsCal::Shared::fOpt = bestFOpt;
			SyncToolsCal::Shared::fOptMutex.unlock();
			firstPass = false;
			parameter.setVectorParameters ( bestX );
			torquesComputation.computeTorquesAndPenalties ( torques, penalties );
			SyncToolsCal::Shared::torqueCalMutex.lock();
			SyncToolsCal::Shared::torqueCal = torques;
			SyncToolsCal::Shared::torqueCalMutex.unlock();
			SyncToolsCal::Shared::vecParamCalMutex.lock();
			SyncToolsCal::Shared::vecParamCal = bestX;
			SyncToolsCal::Shared::vecParamCalMutex.unlock();
		}
		else if ( fOpt < bestFOpt )
		{
			bestFOpt = fOpt;
			bestX = x;
//			std::cout << "Best result: " << bestFOpt << std::endl;
			parameter.setVectorParameters ( bestX );
			torquesComputation.computeTorquesAndPenalties ( torques, penalties );
			SyncToolsCal::Shared::torqueCalMutex.lock();
			SyncToolsCal::Shared::torqueCal = torques;
			SyncToolsCal::Shared::torqueCalMutex.unlock();
			SyncToolsCal::Shared::vecParamCalMutex.lock();
			SyncToolsCal::Shared::vecParamCal = bestX;
			SyncToolsCal::Shared::vecParamCalMutex.unlock();
		}

		// send reply back to sender of the message received above
		if ( nbIteration < executionSimulatedAnnealing.getMaxNoEval() )
		{
			bool rep = false;
			MPI_Send ( &rep, 1, MPI::BOOL, status.MPI_SOURCE, END,
					MPI_COMM_WORLD );
			nbIteration++;
			SyncToolsCal::Shared::mpiIterationMutex.lock();
			SyncToolsCal::Shared::mpiIteration = nbIteration;
			SyncToolsCal::Shared::mpiIterationMutex.unlock();
//			std::cout << "Just launch iteration numbers: " << nbIteration << "."
//					<< std::endl;
		}
		else
		{
			bool rep = true;
			MPI_Send ( &rep, 1, MPI::BOOL, status.MPI_SOURCE, END,
					MPI_COMM_WORLD );
			endProcess++;

			if ( endProcess == ncpu - 1 )
			{
				string XMLfile = inputDataDirectory + "/subjectCalibrated.xml";
				setupData->writeXMLCalibratedFile ( *mySubject, XMLfile );
				break;
			}
		}
	}
	delete mySubject;
}

template <typename T, typename CurveM>
boost::shared_ptr<SetupDataStructure<T, CurveM> > setupSubject ( T& mySubject, string configurationFile )
{
	boost::shared_ptr<SetupDataStructure<T, CurveM> > setupData = boost::shared_ptr<SetupDataStructure<T, CurveM> > ( new SetupDataStructure<T, CurveM> ( configurationFile ) );
	setupData->createCurves();
	setupData->createMuscles ( mySubject );
	setupData->createDoFs ( mySubject );
	setupData->createMusclesNamesOnChannel ( mySubject );
	return setupData;
}
