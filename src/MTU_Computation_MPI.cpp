#ifdef USE_OPENSIM
#include "MuscleAnalyseForSpline_MPI.h"
#include "MuscleAnalyseForSpline.h"
#endif
#include "MTUSplineData.h"
#include "MTUSplineDataWrite.h"
#include <csignal>
#include "mpi.h"
#include "simulatedAnnealing.hxx"
#include <xercesc/util/PlatformUtils.hpp>

void MTUSigintHandler ( int sig )
{
	std::cout << "Quitting..." << std::endl;
	MPI_Finalize();
	exit(0);
}

int main ( int argc, char** argv )
{
#ifdef USE_OPENSIM
	MPI_Init ( &argc, &argv );

	signal ( SIGINT, MTUSigintHandler );

	int ncpu, rank;

	MPI_Comm_size ( MPI_COMM_WORLD, &ncpu );
	MPI_Comm_rank ( MPI_COMM_WORLD, &rank );
	
	std::cout << rank << std::endl;
	std::cout << ncpu << std::endl;

	std::string translateFileName;
	std::string OsimModelName;
	std::string subjectName;
	std::string configurationFile;
	int printOption, nbNode;
	
	xercesc::XMLPlatformUtils::Initialize();

	try
	{
		std::auto_ptr<SimulatedAnnealingType> annealingPointer (
			simulatedAnnealing ( argv[1],
					xml_schema::flags::dont_initialize ) );
		translateFileName = annealingPointer->computeSpline().TranslateFile();
		OsimModelName = annealingPointer->computeSpline().osimFile();
		printOption = annealingPointer->printingOption();
		nbNode = annealingPointer->numberOfNode();
		subjectName = annealingPointer->NameOfSubject();
		configurationFile = annealingPointer->subjectXML();
	}
	catch ( const xml_schema::exception& e )
	{
		cout << e << endl;
		exit ( EXIT_FAILURE );
	}

	MuscleAnalyseForSplineMPI muscleAnalyse ( configurationFile, OsimModelName, translateFileName, nbNode); // Print just coefficients

	muscleAnalyse.computeAnglesStorage();

	muscleAnalyse.run ( rank, ncpu );

	if ( rank == 0 )
	{
		MTUSplineDataWrite splineData ( muscleAnalyse, subjectName );
		splineData.computeTaskCoeffients();
		splineData.writeTaskCoefficients();
	}

	MPI_Finalize();
	xercesc::XMLPlatformUtils::Terminate();
#endif
	return EXIT_SUCCESS;
}
