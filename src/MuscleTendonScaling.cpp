#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <stdlib.h>
#include <fstream>
using std::ofstream;
using std::ifstream;
#include <vector>
using std::vector;
#include <xercesc/util/PlatformUtils.hpp>
#include "simulatedAnnealing.hxx"
#include "ComputePostureControlPoints.h"
#include "ComputeMuscleTendonLength.h"
#include "ComputeMuscleLength.h"
#include "ComputeTSLAndOFL.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "ExecutionSimulatedAnnealing.h"


int main ( int argc, char** argv )
{
	cout << "------------------------------------------------\n";
	cout << " NEUROMUSCOLOKELETAL MODEL: MUSCLE-TENDON-SCALING \n";
	cout << " Calibration using simulated annealing\n";
	cout << "------------------------------------------------\n";

	// COMMAND LINE ARGUMENTS...
	if ( argc < 2 )
	{
		cout << "Usage: calibrate simulatedAnnealingXMLfile\n";
		cout << "Ex; calibrate simulatedAnnealing.xml\n";
		exit ( EXIT_FAILURE );
	}

// 	string emgProccesingFile;
// 	string translateFileName;
// 	string unscaledOsimModelName;
// 	string scaledOsimModelName;
// 	string subjectName;
// 	string configurationFile;
// 	string inputDataDirectory;
// 	int printOption = 0;
// 	int nbNode = 0;
// 	bool useSpline = false;
// 	vector<string> dofsToCalibrate;
// 	vector<string> idTrials;
// 	vector<double> cropMin;
// 	vector<double> cropMax;

	xercesc::XMLPlatformUtils::Initialize();
	
	ExecutionSimulatedAnnealing executionSimulatedAnnealing(argv[1]);

// 	try
// 	{
// 		std::auto_ptr<SimulatedAnnealingType> annealingPointer (
// 			simulatedAnnealing ( argv[1],
// 					xml_schema::flags::dont_initialize ) );
// 		const DofCalSequence& dofSeq = annealingPointer->dofToCalibrate();
// 
// 		for ( DofCalSequence::const_iterator it = dofSeq.begin();
// 				it != dofSeq.end(); it++ )
// 			dofsToCalibrate.push_back ( string ( *it ) );
// 
// 		emgProccesingFile = annealingPointer->EMGProccesing();
// 		useSpline = annealingPointer->computeSpline().use();
// 		translateFileName = annealingPointer->computeSpline().TranslateFile();
// 		scaledOsimModelName = annealingPointer->computeSpline().osimFile();
// 		unscaledOsimModelName = annealingPointer->computeSpline().unscaledOsimFile();
// 		printOption = annealingPointer->printingOption();
// 		nbNode = annealingPointer->numberOfNode();
// 		subjectName = annealingPointer->NameOfSubject();
// 		configurationFile = annealingPointer->subjectXML();
// 		inputDataDirectory = annealingPointer->trials().directory();
// 		const Trials::trial_sequence& trialsSeq = annealingPointer->trials().trial();
// 
// 		for ( Trials::trial_sequence::const_iterator it = trialsSeq.begin();
// 				it != trialsSeq.end(); it++ )
// 		{
// 			idTrials.push_back ( it->trialName() );
// 			cropMin.push_back ( it->cropMinTime() );
// 			cropMax.push_back ( it->cropMaxTime() );
// 		}
// 	}
// 	catch ( const xml_schema::exception& e )
// 	{
// 		cout << e << endl;
// 		exit ( EXIT_FAILURE );
// 	}

	OpenSim::Model unscaledModel ( executionSimulatedAnnealing.getUnscaledOsimFilePreScaling() );
	OpenSim::Model scaledModel ( executionSimulatedAnnealing.getOsimFilePreScaling() );

	MTSS mtss;
	ComputePostureControlPoints cpcp ( executionSimulatedAnnealing.getSubjectXML(), unscaledModel, mtss, executionSimulatedAnnealing.getTranslateFilePreScaling() );

	std::cout << "mtss.size(): " << mtss.size() << std::endl;

	for ( int i = 0; i < mtss.size(); i++ )
		std::cout << mtss.at ( i ).muscleName << "\t";

	std::cout << std::endl;
	std::cout << std::endl;

	for ( int i = 0; i < mtss.front().spanningDOF.size(); i++ )
		std::cout << mtss.front().spanningDOF.at ( i ) << "\t";

	std::cout << std::endl;
	std::cout << std::endl;

	for ( int j = 0; j < mtss.front().spanningDOF.size(); j++ )
	{
		for ( int i = 0; i < mtss.front().postureControlPoints.at ( j ).size(); i++ )
			std::cout << mtss.front().postureControlPoints.at ( j ).at ( i ) << "\t";

		std::cout << std::endl;
	}

	std::cout << std::endl;

	std::cout << mtss.front().optimalPennationAngle << std::endl;
	std::cout << mtss.front().unscaledTendonSlackLength << std::endl;
	std::cout << mtss.front().unscaledOptimalFiberLength << std::endl;

	mtss.clear();
	mtss = MTSS();
	MuscleTendonScalingStruct tibant;
	tibant.muscleName = "tib_ant_r";
	tibant.spanningDOF.push_back ( "ankle_angle_r" );
	std::vector<double> pcp;
	pcp.push_back ( -45.27925256 / 180 * M_PI );
	pcp.push_back ( -39.97929503 / 180 * M_PI );
	pcp.push_back ( -30.43937148 / 180 * M_PI );
	pcp.push_back ( -19.83945642 / 180 * M_PI );
	pcp.push_back ( -10.29953287 / 180 * M_PI );
	pcp.push_back ( 0.30038219 / 180 * M_PI );
	pcp.push_back ( 9.84030574 / 180 * M_PI );
	pcp.push_back ( 20.4402208 / 180 * M_PI );
	pcp.push_back ( 29.98014435 / 180 * M_PI );
	pcp.push_back ( 40.58005941 / 180 * M_PI );
	pcp.push_back ( 44.82002543 / 180 * M_PI );
// 	pcp.push_back(-45 / 180 * M_PI );
// 	pcp.push_back(-40 / 180 * M_PI );
// 	pcp.push_back(-30 / 180 * M_PI );
// 	pcp.push_back(-20 / 180 * M_PI );
// 	pcp.push_back(-10 / 180 * M_PI );
// 	pcp.push_back(0 / 180 * M_PI );
// 	pcp.push_back(10 / 180 * M_PI );
// 	pcp.push_back(20 / 180 * M_PI );
// 	pcp.push_back(30 / 180 * M_PI );
// 	pcp.push_back(40 / 180 * M_PI );
// 	pcp.push_back(45 / 180 * M_PI );
	tibant.postureControlPoints.push_back ( pcp );
// 	tibant.fiberLength.push_back ( 0.434584 );
// 	tibant.fiberLength.push_back ( 0.475348 );
// 	tibant.fiberLength.push_back ( 0.502402 );
// 	tibant.fiberLength.push_back ( 0.564572 );
// 	tibant.fiberLength.push_back ( 0.564572 );
// 	tibant.fiberLength.push_back ( 0.619529 );
// 	tibant.fiberLength.push_back ( 0.690720 );
// 	tibant.fiberLength.push_back ( 0.690720 );
// 	tibant.fiberLength.push_back ( 0.903835 );
// 	tibant.fiberLength.push_back ( 1.022462 );
// 	tibant.fiberLength.push_back ( 1.051177 );
// 	tibant.muscleTendonLength.push_back ( 0.34396808 );
// 	tibant.muscleTendonLength.push_back ( 0.34111341 );
// 	tibant.muscleTendonLength.push_back ( 0.3355193 );
// 	tibant.muscleTendonLength.push_back ( 0.32867351 );
// 	tibant.muscleTendonLength.push_back ( 0.32201543 );
// 	tibant.muscleTendonLength.push_back ( 0.3141642 );
// 	tibant.muscleTendonLength.push_back ( 0.3067985 );
// 	tibant.muscleTendonLength.push_back ( 0.29844868 );
// 	tibant.muscleTendonLength.push_back ( 0.2909945 );
// 	tibant.muscleTendonLength.push_back ( 0.2831612 );
// 	tibant.muscleTendonLength.push_back ( 0.28029887 );
	tibant.optimalPennationAngle = 0.09000000;
	tibant.unscaledTendonSlackLength = 0.22300000;
	tibant.unscaledOptimalFiberLength = 0.09800000;

	mtss.push_back ( tibant );

	for ( int j = 0; j < mtss.at ( 0 ).spanningDOF.size(); j++ )
	{
		for ( int i = 0; i < mtss.at ( 0 ).postureControlPoints.at ( j ).size(); i++ )
			std::cout << mtss.at ( 0 ).postureControlPoints.at ( j ).at ( i ) << "\t";

		std::cout << std::endl;
	}

	ComputeMuscleTendonLength cmtl ( unscaledModel, scaledModel, mtss, executionSimulatedAnnealing.getTranslateFilePreScaling() );
	cmtl.run();
// 	for(int j =0; j < mtss.size(); j++)
// 	{
// 		for(int i =0; i < mtss.at(j).muscleTendonLength.size(); i++)
// 			std::cout << mtss.at(j).muscleTendonLength.at(i) << "\t";
// 		std::cout << std::endl;
// 	}
// 	std::cout << std::endl;

	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 0 ) = 0.3323;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 1 ) = 0.3297;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 2 ) = 0.3240;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 3 ) = 0.3177;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 4 ) = 0.3110;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 5 ) = 0.3038;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 6 ) = 0.2963;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 7 ) = 0.2888;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 8 ) = 0.2813;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 9 ) = 0.2742;
	mtss.at ( 0 ).unscaledMuscleTendonLength.at ( 10 ) = 0.2711;



	ComputeMuscleLength cml ( executionSimulatedAnnealing.getSubjectXML(), mtss );
	cml.run();

	for ( int j = 0; j < mtss.size(); j++ )
	{
		std::cout << mtss.at ( j ).muscleName << std::endl;
		std::cout << "UnscaledMuscleTendonLength" << "\t";

		for ( int i = 0; i < mtss.at ( j ).unscaledMuscleTendonLength.size(); i++ )
			std::cout << mtss.at ( j ).unscaledMuscleTendonLength.at ( i ) << "\t";

		std::cout << std::endl;
		std::cout << "fiberLength" << "\t\t";

		for ( int i = 0; i < mtss.at ( j ).fiberLength.size(); i++ )
			std::cout << mtss.at ( j ).fiberLength.at ( i ) << "\t";

		std::cout << std::endl;
		std::cout << std::endl;
	}

	std::cout << std::endl;

	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 0 ) = 0.34396808;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 1 ) = 0.34111341;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 2 ) = 0.3355193;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 3 ) = 0.32867351;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 4 ) = 0.32201543;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 5 ) = 0.3141642;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 6 ) = 0.3067985;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 7 ) = 0.29844868;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 8 ) = 0.2909945;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 9 ) = 0.2831612;
	mtss.at ( 0 ).scaledMuscleTendonLength.at ( 10 ) = 0.28029887;


	ComputeTSLAndOFL ctslofl ( executionSimulatedAnnealing.getSubjectXML(), mtss );
	ctslofl.run();

	for ( int j = 0; j < mtss.size(); j++ )
	{
		std::cout << mtss.at ( j ).muscleName << std::endl;
		std::cout << "unnscaledTendonSlackLength: " << mtss.at ( j ).unscaledTendonSlackLength << "\tunscaledOptimalFiberLength: " << mtss.at ( j ).unscaledOptimalFiberLength << std::endl;
		std::cout << "scaledTendonSlackLength: " << mtss.at ( j ).scaledTendonSlackLength << "\tscaledOptimalFiberLength: " << mtss.at ( j ).scaledOptimalFiberLength << std::endl;
		std::cout << std::endl;
	}

	std::cout << std::endl;

}

#endif
