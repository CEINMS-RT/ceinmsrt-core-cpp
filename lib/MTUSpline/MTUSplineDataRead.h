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

#ifndef MTUSPLINEDATAREAD_H_
#define MTUSPLINEDATAREAD_H_

#include "MTUSplineData.h"
#include "MTUSplineBase.h"
//#include <boost/shared_ptr.hpp>
#include <memory>
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <map>
using std::map;
#include <set>
using std::set;

#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>

#include "NMSmodel.h"

#include <fstream>
using std::ifstream;
using std::ofstream;
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iomanip>

/**
 * Class for creating MTU spline from coefficients files created by MTUSplineDataWrite class.
 */
class MTUSplineDataRead: public MTUSplineData
{
public:

	struct Task
	{
		set<string> uniqueDOFlist;
		vector<string> uniqueMuscleList;
		vector< std::shared_ptr<MTUSplineBase> > splines_;
	};

	MTUSplineDataRead();
	MTUSplineDataRead(string& configurationFile, const string& subjectName);
	virtual ~MTUSplineDataRead();

	/**
	 * read coefficients and create MTU spline by task (a group of DOF).
	 */
	void readTaskCoefficients();
	void readCoefficients(const string& inputCoeffFilename);

	inline vector<Task> getTaskVect()
	{
		return vectTask_;
	}

	inline vector<string> getMuscleOnDof(const string& dofName)
	{
		vector<string> muscleList;
		for (DOFMuscleMap::left_const_iterator itBimap = DOFMuscleConnect_.left.lower_bound(dofName);
						itBimap != DOFMuscleConnect_.left.upper_bound(dofName); itBimap++)
		{
			muscleList.push_back(itBimap->second);
		}
		return muscleList;
	}

	/*
	 * Compute lmt by task.
	 * @param betweenNode True compute for between node False compute for on node.
	 */
	void evalTaskLmt(bool betweenNode);

	/*
	 * Compute Ma by task.
	 * @param betweenNode True compute for between node False compute for on node.
	 */
	void evalTaskMa(bool betweenNode);
private:
	/**
	 * Recursive function for writing all the coefficients of the first phase spline
	 * and second phase spline. Recall himself for writing the first phase spline
	 * and second phase spline for the N-1 DOF
	 * @param myStream Line containing all the coefficients.
	 * @param FirstPhase Spline of the first phase.
	 * @param secondPhaseCoefficients Coefficients of the second phase spline (1D).
	 * @param cptRecur Recursive enumerator.
	 */
	template<class T>
	void readCoefficientsFirstPhase(stringstream& myStream, T& FirstPhase, std::vector<double>& secondPhaseCoefficients, int& cptRecur);

	typedef boost::bimap<boost::bimaps::multiset_of<string>, boost::bimaps::multiset_of<string> > DOFMuscleMap;
	typedef DOFMuscleMap::value_type position;

	DOFMuscleMap DOFMuscleConnect_;

	vector<string> vectMuscleNameList_;
	vector<string> vectDOFNameList_;
	map<string, std::shared_ptr<MTUSplineBase> > mapMuscleToSpline_;
	vector<std::shared_ptr<MTUSplineBase> > spline_;
	vector<Task> vectTask_;

};

#endif /* MTUSPLINEDATAREAD_H_ */
