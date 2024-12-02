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

#ifdef USE_GUI
#include <mainwindow.h> // always on top in windows
#include <QApplication>
#include <QMainWindow>
#include <QIcon>
#endif
#include "ModelEvaluationRealTime.h"

#include "CommonCEINMS.h"
#include "EMGFromFile.h"
#include "LmtMaFromFile.h"
#include "EMGFromDevice.h"
#include "AngleFromDevice.h"
#include "LmtMaFromMTUSpline.h"
#include "ExternalTorqueFromFile.h"
// #include "ModelEvaluationOnline.h"
// #include "ModelEvaluationOffline.h"
// #include "ModelEvaluationHybrid.h"

#include "SetupDataStructure.h"
#include "Activation/ExponentialActivation.h"
#include "Activation/ExponentialActivationRT.h"
#include "Tendon/StiffTendon.h"
#include "Tendon/ElasticTendon.h"
#include "Tendon/ElasticTendon_BiSec.h"
// #include "ErrorMinimizerAnnealing.h"
// #include "HybridWeightings.h"
#include "Curve.h"
#include "ExecutionXmlReader.h"
#include "DynLib.h"
#include <ctime>
#include <cstdlib>

#include "executionIK_ID.hxx"

#include <tclap/CmdLine.h>

#define ACE_GCC_HAS_TEMPLATE_INSTANTIATION_VISIBILITY_ATTRS 1

//#define VERBOSE

#include <boost/thread.hpp>
#include "boost/program_options.hpp"
#include <xercesc/util/PlatformUtils.hpp>

#ifdef UNIX
#include <unistd.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

/**
 * TODO: add Hybrid model
 * 		Joint Compression
 *
 */

void CEINMSSigintHandler(int sig);
void timerToEndCEINMS(double timer);

std::string trialNameCommandLine;

template<typename T1, typename T2, typename T3, typename T4, typename T5>  //add another parameter (timer) to threads
void runThreads(T1& t1, T2& t2, T3& t3, T4& t4, T5& t5);

#ifdef USE_GUI
template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>  //add another parameter (timer) to threads
void runThreadsWithGui(T1& t1, T2& t2, T3& t3, T4& t4, T5& t5, T6& t6, QSplashScreen& splash);
#endif

template <typename T, typename CurveM>
void setupSubject(T& mySubject, string configurationFile);

void printHeader();

void printAuthors();

void CLIOption(const int& argc, char** argv, std::string& exect, bool& exectFound, std::string& subt, bool& subtFound, bool& gui, int& verbose,
	std::string& recordDirectory, bool& record, std::string& processDirectory, bool& process, double& timer);

int main(int argc, char** argv)
{
	//** COMAND LINE OPTION **//
	std::string exect = "cfg/TestData/executionRT_OnlineCal.xml";
	bool exectFound = false;
	std::string subt = "cfg/TestData/gait2392Left.xml";
	bool subtFound = false;
	bool gui = true;
	int verbose = 1;
	std::string recordDirectory = "Output";
	bool record = false;
	std::string processDirectory;
	bool process = false;
	double timer = 0.0;  // initilize timer constraints for CEINMS

	signal(SIGINT, CEINMSSigintHandler);

	CLIOption(argc, argv, exect, exectFound, subt, subtFound, gui, verbose, recordDirectory, record, processDirectory, process, timer);

#ifdef USE_GUI
	//** GUI INIT **//
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	QApplication::setGraphicsSystem("raster");

#endif
	//QCoreApplication::addLibraryPath("./");
	/*QCoreApplication::addLibraryPath("./bin/Win/Release");
	QCoreApplication::addLibraryPath("./bin/Win/Release/platforms");
	QCoreApplication::addLibraryPath("C:/Users/DurandauGV/Documents/CEINMS/CEINMS-RT/trunk/bin/Win/Release/platforms");
	QCoreApplication::addLibraryPath("C:/Users/DurandauGV/Documents/CEINMS/CEINMS-RT/trunk/bin/Win/Release");*/

	QApplication a(argc, argv);
	QPixmap pixmap("CEINMS-RT_V2.png");
	QSplashScreen splash(pixmap);

	if (gui)
	{
		delete InterThread::readyToStart;
		InterThread::readyToStart = new boost::barrier(SyncTools::Shared::numberOfThreads + 1);
		a.setWindowIcon(QIcon("CEINMS-RT_V2_ICON.png"));
		splash.show();
	}

#endif

#ifdef VERBOSE

	//** COMmAND LINE TEXT **//
	if (verbose > 0)
	{
		printHeader();
		printAuthors();
	}

#endif
	

	InterThread::setSyncGui(gui);
	InterThread::setSyncVerbose(verbose);

	xercesc::XMLPlatformUtils::Initialize(); // for the thread safe of the parsing of the XML use xml_schema::flags::dont_initialize when parsing.

	//** LOAD XSD SUBJECT FOR XML **//
	string configurationFile(subt);

	try
	{
		std::auto_ptr<NMSmodelType> subjectPointer(subject(subt, xml_schema::flags::dont_initialize));
	}
	catch (const xml_schema::exception& e)
	{
		COUT << e <<std::endl;
		exit(EXIT_FAILURE);
	}

	//** LOAD EXECUTION XML **//
	ExecutionXmlReader executionCfg(exect);

	if (gui)
	{
		try
		{
			std::auto_ptr<ExecutionIKType> executionIKPointer(executionIK(executionCfg.getAngleFile(), xml_schema::flags::dont_initialize));
			//ExecutionIKType::OsimFile_type& myOsimModel(executionIKPointer->OsimFile());
			std::string OsimName = executionIKPointer->OsimFile();
			COUT << OsimName << std::endl;
			InterThread::setModelFileName(OsimName);
		}
		catch (const xml_schema::exception& e)
		{
			COUT << e <<std::endl << std::flush;
			exit(EXIT_FAILURE);
		}
	}

	//** ANGLE AND COMSUMER PLUGION **//
	DynLib<AngleAndComsumerPlugin> plugin;
	DynLib<EmgAndAngleAndComsumerPlugin> EACplugin;
	bool useOfAngleAndComsumerPlugin = executionCfg.useOfAngleAndComsumerPlugin();
	bool useOfEmgAndAngleAndComsumerPlugin = false;

	if (executionCfg.useOfEmgAndAngleAndComsumerPlugin() && !process)
	{
		EACplugin.setDynLib(executionCfg.getEmgAndAngleAndComsumerPlugin());
		useOfAngleAndComsumerPlugin = false;
		useOfEmgAndAngleAndComsumerPlugin = true;
	}

	if (useOfAngleAndComsumerPlugin && !process)
	{
		plugin.setDynLib(executionCfg.getAngleAndComsumerPlugin());
	}
	

	//** CLASS FOR GETTING EMG ANGLE AND THE MTUSPLINE (RUN IN DIFFERENT THREAD) **//
	AngleFromDevice angleProducer(subt, exect, record, recordDirectory, processDirectory, process); //not used anymore to delete?
	EMGFromDevice emgProducerDevice(subt, exect, record, recordDirectory, processDirectory, process);//not used anymore to delete?
	LmtMaFromMTUSpline lmtMaProducerSpline(subt, executionCfg.getNameOfSubject(), exect, record, recordDirectory, processDirectory, process);

	//** IF WE USE ANGLE AND CONSUMER PLUGIN TYPICALLY EXOSKELETON **//
	if (useOfAngleAndComsumerPlugin && !process)
	{
		if (record)
			plugin.getPlugin()->setDirectory(recordDirectory);

		plugin.getPlugin()->setVerbose(verbose);
		plugin.getPlugin()->setRecord(record);

		//plugin.getPlugin()->init(atoi(executionCfg.getComsumerPort().c_str()));

		//** SET THE PLUGIN FOR ANGLE AND BSPLLINE THREAD BECAUSE IS THE SAME PLUGIN THTA THEY HAVE TO SHARE **//
		//std::cout << "start AAC 1" << std::endl;
		angleProducer.setPluginAAC(plugin.getPlugin());
		lmtMaProducerSpline.setPluginAAC(plugin.getPlugin());
	}
	else if (useOfEmgAndAngleAndComsumerPlugin && !process)
	{
		if (record)
			EACplugin.getPlugin()->setDirectory(recordDirectory);

		EACplugin.getPlugin()->setVerbose(verbose);
		EACplugin.getPlugin()->setRecord(record);

		//plugin.getPlugin()->init(atoi(executionCfg.getComsumerPort().c_str()));

		//** SET THE PLUGIN FOR ANGLE AND BSPLLINE THREAD BECAUSE IS THE SAME PLUGIN THTA THEY HAVE TO SHARE **//
		//std::cout << "start AAC 1" << std::endl;
		angleProducer.setPluginEAC(EACplugin.getPlugin());
		lmtMaProducerSpline.setPluginEAC(EACplugin.getPlugin());
	}

	//** GET THE NMS MODEL SPECIFICATION FOR THE TEMPLATE **//
	NMSModelCfg::RunMode runMode = executionCfg.getRunMode();

	switch (runMode)
	{
	case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOnline:
	{
#ifdef VERBOSE

		if (verbose > 1)
			COUT << "Model: RealTimeOpenLoopExponentialActivationStiffTendonOnline." <<std::endl;

#endif

		typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
		MyNMSmodel mySubject;
		setupSubject<MyNMSmodel, Curve<CurveMode::Online> >(mySubject, subt);
		ModelEvaluationRealTime<MyNMSmodel> consumer(subt, mySubject, exect, record, recordDirectory, processDirectory, process);

		if (useOfAngleAndComsumerPlugin && !process)
		{
			//std::cout << "start AAC 2" << std::endl;
			consumer.setPluginAAC(plugin.getPlugin());
		}
		else if (useOfEmgAndAngleAndComsumerPlugin && !process)
		{
			consumer.setPluginEAC(EACplugin.getPlugin());
		}
		

#ifdef USE_GUI

		if (gui)
			runThreadsWithGui(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, a, timer, splash);  // pass timer to thread
		else
			runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread

#else
		runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread
#endif
		break;
	}

	case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOffline:
	{
#ifdef VERBOSE

		if (verbose > 1)
			COUT << "Model: RealTimeOpenLoopExponentialActivationStiffTendonOffline." <<std::endl;

#endif

		typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> MyNMSmodel;
		MyNMSmodel mySubject;
		setupSubject<MyNMSmodel, Curve<CurveMode::Offline> >(mySubject, subt);
		ModelEvaluationRealTime<MyNMSmodel> consumer(subt, mySubject, exect, record, recordDirectory, processDirectory, process);

		if (useOfAngleAndComsumerPlugin && !process)
		{
			//std::cout << "start AAC 2" << std::endl;
			consumer.setPluginAAC(plugin.getPlugin());
		}
		else if (useOfEmgAndAngleAndComsumerPlugin && !process)
		{
			consumer.setPluginEAC(EACplugin.getPlugin());
		}

#ifdef USE_GUI

		if (gui)
			runThreadsWithGui(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, a, timer, splash);  // pass timer to thread
		else
			runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread

#else
		runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread
#endif
		break;
	}

	case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline:
	{
#ifdef VERBOSE

		if (verbose > 1)
			COUT << "Model: RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline." <<std::endl;

#endif

		typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
		MyNMSmodel mySubject;
		setupSubject<MyNMSmodel, Curve<CurveMode::Online> >(mySubject, subt);
		ModelEvaluationRealTime<MyNMSmodel> consumer(subt, mySubject, exect, record, recordDirectory, processDirectory, process);

		if (useOfAngleAndComsumerPlugin && !process)
			consumer.setPluginAAC(plugin.getPlugin());
		else if (useOfEmgAndAngleAndComsumerPlugin && !process)
		{
			consumer.setPluginEAC(EACplugin.getPlugin());
		}

#ifdef USE_GUI

		if (gui)
			runThreadsWithGui(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, a, timer, splash);  // pass timer to thread
		else
			runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread

#else
		runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread
#endif
		break;
	}



	case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOnline:
	{
#ifdef VERBOSE

		if (verbose > 1)
			COUT << "Model: RealTimeOpenLoopExponentialActivationElasticTendonOnline." <<std::endl;

#endif

		typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
		MyNMSmodel mySubject;
		setupSubject<MyNMSmodel, Curve<CurveMode::Online> >(mySubject, subt);
		ModelEvaluationRealTime<MyNMSmodel> consumer(subt, mySubject, exect, record, recordDirectory, processDirectory, process);

		if (useOfAngleAndComsumerPlugin && !process)
			consumer.setPluginAAC(plugin.getPlugin());
		else if (useOfEmgAndAngleAndComsumerPlugin && !process)
		{
			consumer.setPluginEAC(EACplugin.getPlugin());
		}

#ifdef USE_GUI

		if (gui)
			runThreadsWithGui(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, a, timer, splash);  // pass timer to thread
		else
			runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread

#else
		runThreads(consumer, emgProducerDevice, lmtMaProducerSpline, angleProducer, timer);  // pass timer to thread
#endif
		break;
	}

	default:
	{
		COUT << runMode <<std::endl;
		COUT << "Implementation not available yet. Verify you XML configuration file\n";
		break;
	}
	}
	
	//** STOP THE PLUGIN ANGLE AND CONSUMER **//
	if (useOfAngleAndComsumerPlugin && !process)
	{
		plugin.getPlugin()->stop();
	}
	else if (useOfEmgAndAngleAndComsumerPlugin && !process)
	{
		EACplugin.getPlugin()->stop();
	}

	delete InterThread::readyToStart;

	xercesc::XMLPlatformUtils::Terminate();

	return 0;
}


// Removes single and double quotes from input parameters, as they must be scaped in case there are spaces, for example
std::string cleanString(const std::string& inputString){
	std::string outString = inputString;
	outString.erase(std::remove(outString.begin(), outString.end(), '\''), outString.end() );
	outString.erase(std::remove(outString.begin(), outString.end(), '\"'), outString.end() );
	return outString;
}

void CLIOption(const int& argc, char** argv, std::string& exect, bool& exectFound, std::string& subt, bool& subtFound, bool& gui, int& verbose,
	std::string& recordDirectory, bool& record, std::string& processDirectory, bool& process, double& timer)
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
		TCLAP::ValueArg<std::string> nameArgS("s", "subject", "Subject specific for CEINMS xml file. See subject.xsd in XSD directory for more information.", true, "default", "string");

		TCLAP::ValueArg<std::string> nameArgE("e", "execution", "Execution xml file option. See execution.xsd in XSD directory for more information.", true, "default", "string");

		TCLAP::ValueArg<int> nameArgV("v", "verbose", "Verbose option. arg (int) is the level of verbose output (0 no output, 1 basic output, 2 debug information and 3 in-loop debug).", false, 0, "int");

		TCLAP::ValueArg<std::string> nameArgR("r", "record", "Save the output in a directory. The name of the directory is arg (string).", false, "NoRecord", "string");

		TCLAP::ValueArg<std::string> nameArgP("p", "process", "Process the emgFilt.txt and ik.sto found in arg (string) directory in a offline ways. This option overrule the execution.xml.", false, "NoProcess", "string");

		// timer to kill ceinms. initilize as 0 and takes any double number. If it is larger than 0, then kill CEINMS after the input number of seconds.
		TCLAP::ValueArg<double> nameArgT("t", "timer", "Stop CEINMS after a certain period of time (seconds). This option has the similar effect as ctrl + C when time limit reached (0: no timing limit, other positive double number for 'n' seconds waiting time)", false, 0, "double");

		// Add the argument nameArg to the CmdLine object. The CmdLine object
		// uses this Arg to parse the command line.
		cmd.add(nameArgS);
		cmd.add(nameArgE);
		cmd.add(nameArgV);
		cmd.add(nameArgR);
		cmd.add(nameArgP);
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
		subt = cleanString(nameArgS.getValue());
		exect = cleanString(nameArgE.getValue());
		gui = guiCMD.getValue();
		verbose = nameArgV.getValue();
		processDirectory = cleanString(nameArgP.getValue());
		recordDirectory = cleanString(nameArgR.getValue());
		timer = nameArgT.getValue(); // get the input sleep time values, if given

		if (processDirectory == "NoProcess")
			process = false;
		else
			process = true;

		if (recordDirectory == "NoRecord")
			record = false;
		else
			record = true;

	}
	catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
		exit(0);
	}
	
}

void CEINMSSigintHandler(int sig)
{
	InterThread::setEndThread(true);
}

void timerToEndCEINMS(double timer)  // function to kill CEINMS without Gui, when time is out 
{

#ifdef _WIN32  
	if (timer > 0.0)  // only works if timer is larger than 0
	{
		std::cout << "Kill CEINMS after : " << timer << " seconds" << '\n';
		Sleep(timer * 1000);  // takes milliseconds in windows
		InterThread::setEndThread(true);  // stop ceinms thread
	}
#else
	if (timer > 0.0)  // only works if timer is larger than 0
	{
		std::cout << "Kill CEINMS after : " << timer << " seconds" << '\n';
		usleep(timer * 1000 * 1000); // takes microseconds in unix
		InterThread::setEndThread(true);  // stop ceinms thread
	}
#endif

}

void timerToEndCEINMSwithGui(double timer, MainWindow &gui)  // function to kill CEINMS with Gui, when time is out 
{
	
#ifdef _WIN32
	if (timer > 0.0)
	{
		std::cout << "Kill CEINMS after : " << timer << " seconds" << '\n';
		Sleep(timer*1000);  // takes milliseconds in windows
		InterThread::setEndThread(true);  // stop ceinms thread
		gui.close();  // kill gui
	}
#else
	if (timer > 0.0)
	{
		std::cout << "Kill CEINMS after : " << timer << " seconds" << '\n';
		usleep(timer*1000*1000); // takes microseconds in unix
		InterThread::setEndThread(true);   // stop ceinms thread
		gui.close();  // kill gui
	}
#endif

}

template<typename T1, typename T2, typename T3, typename T4, typename T5>
void runThreads(T1& t1, T2& t2, T3& t3, T4& t4, T5& t5)
{
	boost::thread thread1(boost::ref(t1)); // consumer: 1 thread
	boost::thread thread3(boost::ref(t3)); // lmtMaProducerSpline: 1 thread
	boost::thread timerthread(&timerToEndCEINMS, t5); // timer to kill ceinms: 1 thread
	thread1.join();
	thread3.join();
	timerthread.join();
}

#ifdef USE_GUI
template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
void runThreadsWithGui(T1& t1, T2& t2, T3& t3, T4& t4, T5& t5, T6& t6, QSplashScreen& splash)
{
	bool killGui = false;

	std::cout << "killGui: " << killGui << '\n';

	boost::thread thread1(boost::ref(t1)); // consumer: 1 thread
	boost::thread thread3(boost::ref(t3)); // lmtMaProducerSpline: 1 thread
	
	
	MainWindow gui;

	boost::thread timerthread(&timerToEndCEINMSwithGui, t6, boost::ref(gui));  // timer to kill ceinms: 1 thread

	gui.addEMG();
	gui.addTorqueCEINMS();
	gui.addTorqueID();
	gui.addMuscleForceBar();
	gui.addDemo();
	gui.add3DIK();
	gui.addTimingPanel();
	gui.start();
	splash.finish(&gui);
	t5.exec();

	thread1.join();
	thread3.join();
	timerthread.join();

}
#endif

template <typename T, typename CurveM>
void setupSubject(T& mySubject, string configurationFile)
{
	SetupDataStructure<T, CurveM> setupData(configurationFile);
	setupData.createCurves();
	setupData.createMuscles(mySubject);
	setupData.createDoFs(mySubject);
	setupData.createMusclesNamesOnChannel(mySubject);
}

void printHeader()
{
	cout <<std::endl;
#ifdef UNIX
	cout << "\033[0;33m+-+-+-+-+-+-+-+-+-+\n"
		<< "|C|E|I|N|M|S|-|R|T|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|C|a|l|i|b|r|a|t|e|d|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|E|M|G|-|I|n|f|o|r|m|e|d|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|N|e|u|r|o|m|u|s|c|u|l|o|s|k|e|l|e|t|a|l|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|R|e|a|l|-|t|i|m|e|\n"
		<< "+-+-+-+-+-+-+-+-+-+\n"
		<< "|T|o|o|l|b|o|x|\n"
		<< "+-+-+-+-+-+-+-+\n"
		<< "|L|I|N|U|X|\n"
		<< "+-+-+-+-+-+-+-+\033[0m\n\n";
#endif
#ifdef WIN32
	cout << "+-+-+-+-+-+-+-+-+-+\n"
		<< "|C|E|I|N|M|S|-|R|T|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|C|a|l|i|b|r|a|t|e|d|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|E|M|G|-|I|n|f|o|r|m|e|d|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|N|e|u|r|o|m|u|s|c|u|l|o|s|k|e|l|e|t|a|l|\n"
		<< "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
		<< "|R|e|a|l|-|t|i|m|e|\n"
		<< "+-+-+-+-+-+-+-+-+-+\n"
		<< "|T|o|o|l|b|o|x|\n"
		<< "+-+-+-+-+-+-+-+\n"
		<< "|W|I|N|D|O|W|S|\n"
		<< "+-+-+-+-+-+-+-\n";
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

