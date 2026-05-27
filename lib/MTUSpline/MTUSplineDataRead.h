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
