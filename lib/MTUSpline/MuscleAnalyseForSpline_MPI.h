// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
