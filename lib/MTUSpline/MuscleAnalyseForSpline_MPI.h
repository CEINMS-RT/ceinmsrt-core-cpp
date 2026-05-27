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

#ifndef MUSCLEANALYSEFORSPLINE_MPI_H_
#define MUSCLEANALYSEFORSPLINE_MPI_H_

// #define USE_OPENSIM

#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include "SetupDataStructure.h"
#include "TranslateOpenSimCEINMS.h"
#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <stdlib.h>
#include<map>
#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <cmath>
#include <algorithm>
#include <set>
#include <boost/shared_ptr.hpp>

#ifdef UNIX
#include <unistd.h>
#endif
#include <ios>
#include <fstream>

#ifdef MPI
#include "mpi.h"
#endif

#include "MuscleAnalyseForSpline.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/*
 * Class for computing the DOF groups, angles, Lmt and Ma using OpenSim.
 */
class MuscleAnalyseForSplineMPI: public MuscleAnalyseForSpline
{
public:

	
		MuscleAnalyseForSplineMPI ( string& configurationFile, string& OsimModelName,
				int nbOfStep);
		MuscleAnalyseForSplineMPI ( string& configurationFile, string& OsimModelName,
				const string& translateFileName, int nbOfStep);
	virtual ~MuscleAnalyseForSplineMPI();

	
	/**
	 * Run the opensim muscle analysis.
	 * Call after the computeAnglesStorage method.
	 */
	void run(const int& rank, const int& ncpu);


protected:
	
	enum mpi_const {DATA_REQ, DATA_RES};

	void setDOF(const Task& task);
};
#endif

#endif /* MUSCLEANALYSEFORSPLINE_H_ */
