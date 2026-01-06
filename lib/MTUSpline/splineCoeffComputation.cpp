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

#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include "SetupDataStructure.h"
#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <stdlib.h>
#include "MuscleAnalyseForSpline.h"
#include "MTUSplineData.h"
#include "MTUSplineDataWrite.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

bool legal_int(const char *str)
{
    while (*str)
        if (!isdigit(*str++))
            return false;
    return true;
}

int parseLine(char* line){
    int i = strlen(line);
    while (*line < '0' || *line > '9') line++;
    line[i-3] = '\0';
    i = atoi(line);
    return i;
}

void printMemUsage()
{
	FILE* file = fopen("/proc/self/status", "r");
	int result = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL){
		if (strncmp(line, "VmRSS:", 6) == 0){
			result = parseLine(line);
			break;
		}
	}

	cout << "Virtual Memory usage: " << result;

	while (fgets(line, 128, file) != NULL){
		if (strncmp(line, "VmSize:", 7) == 0){
			result = parseLine(line);
			break;
		}
	}
	fclose(file);

	cout << "; Physical memory usage: " << result << endl;
}


int main(int argc, const char* argv[])
{
	cout << "----------------------------------------------------\n";
	cout << " SIMPLE PROGRAM TO COMPUTE THE COEFFICIENTS FOR THE SPLINE\n";
	cout << " Copyright (C) 2015 Guillaume Durandau, Massimo Sartori\n";
	cout << "----------------------------------------------------\n";


	int printOption;
	string OsimModelName;
	try
	{
		OsimModelName = string(argv[1]);
	}
	catch (const exception& e)
	{
		cout << "Usage: splineCoeffComputation OsimModel SubjectSpecificXml NumberOfNode TranslateFile NameOfSubject printingOption \n" << endl;
		cout << "The printing option are:";
		cout << "							- Nothing or 0: no printing on files" << endl;
		cout << "							- 1: print .in file (CEINMS)" << endl;
		cout << "							- 2: print .mot file (OpenSim)" << endl;
		cout << "							- 3: print .in file (CEINMS) and .mot file (OpenSim)" << endl;
		cout << "For showing this message use: splineCoeffComputation -help " << endl;
		exit(EXIT_FAILURE);
	}

	cout << argc << endl;

	// Check command line arguments
	if (argc < 6 or OsimModelName == "h" or OsimModelName == "H" or OsimModelName == "help" or OsimModelName == "Help" or OsimModelName == string("-h") or OsimModelName == string("-H") or OsimModelName == string("-help") or OsimModelName == string("-Help"))
	{
		cout << "Usage: splineCoeffComputation OsimModel SubjectSpecificXml NumberOfNode TranslateFile NameOfSubject printingOption \n" << endl;
		cout << "The printing option are:";
		cout << "							- Nothing or 0: no printing on files" << endl;
		cout << "							- 1: print .in file (CEINMS)" << endl;
		cout << "							- 2: print .mot file (OpenSim)" << endl;
		cout << "							- 3: print .in file (CEINMS) and .mot file (OpenSim)" << endl;
		cout << "For showing this message use: splineCoeffComputation -help " << endl;
		exit(EXIT_FAILURE);
	}
	else if(argc == 7)
	{
		if (!legal_int(argv[6]))
		{
			cout << "Option printingOption need an integer." << endl;
			exit(0);
		}
		else
		{
			printOption = atoi(argv[6]);
		}
	}
	else if(argc == 6)
	{
		printOption = 0;
	}


	string configurationFile(argv[2]);
	string translateFileName(argv[4]);

	cout << translateFileName << endl;

	if (!legal_int(argv[3]))
	{
		cout << "Option printingOption need an integer." << endl;
		exit(0);
	}

	MuscleAnalyseForSpline muscleAnalyse(configurationFile, OsimModelName, translateFileName, atoi(argv[3]), printOption);

	muscleAnalyse.computeAnglesStorage();

	muscleAnalyse.run(false);
	if(printOption == 1 || printOption == 3)
		muscleAnalyse.run(true);

	muscleAnalyse.writeLmt();
	muscleAnalyse.writeMa();

	string modelName(argv[5]);

	MTUSplineDataWrite splineData(muscleAnalyse, modelName);
	splineData.computeTaskCoeffients();

	splineData.writeTaskCoefficients();

	printMemUsage();
}

