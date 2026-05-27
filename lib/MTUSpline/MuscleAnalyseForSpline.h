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

#ifndef MUSCLEANALYSEFORSPLINE_H_
#define MUSCLEANALYSEFORSPLINE_H_

#include "CommonCEINMS.h"
#ifdef USE_OPENSIM
#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include "SetupDataStructure.h"
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
#include <boost/filesystem.hpp>

#ifdef UNIX
#include <unistd.h>
#endif
#include <ios>
#include <fstream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/*
 * Class for computing the DOF groups, angles, Lmt and Ma using OpenSim.
 */
class MuscleAnalyseForSpline
{
	public:

		struct Task
		{
			Storage angleStorage;
			Storage angleBetweenNodeStorage;
			set<string> uniqueDOFlist;
			//set<string> uniqueMuscleList;
			vector<string> uniqueMuscleList;
			vector< vector< double > > lmtVectorMat;/*!< Muscle tendon length vector computed by OpenSim*/
			vector< vector< vector< double > > > maVector3DMat;/*!< Moment arm vector computed by OpenSim. DOF then muscle*/
			vector< vector< double > > lmtVectorMatBetweenNode;/*!< Muscle tendon length vector computed by OpenSim*/
			vector< vector< vector< double > > > maVector3DMatBetweenNode;/*!< Moment arm vector computed by OpenSim. DOF then muscle*/
		};

		MuscleAnalyseForSpline() {};

		MuscleAnalyseForSpline ( const string& configurationFile, const string& OsimModelName,
				const int& nbOfStep, const int& printOption = 0 );
		MuscleAnalyseForSpline ( const string& configurationFile, const string& OsimModelName,
				const string& translateFileName, const int& nbOfStep, const int& printOption = 0 );
		virtual ~MuscleAnalyseForSpline();

		/**
		 * Compute the storage for the muscle analysis.
		 * The nbOfStep represent the number of value compute for each DOF
		 * (min range to max range of the DOF from the osim file).
		 * The time step is 10 ms.
		 * @param nbOfStep number of step
		 */
		void computeAnglesStorage();

		/**
		 * Run the opensim muscle analysis.
		 * Call after the computeAnglesStorage method.
		 */
		void run ( bool betweenNode );

		inline const vector<string>& getMuscleNameVectorCEINMS() const
		{
			return vectMuscleNameListOpenSim_;
		}

		inline const vector<string>& getDOFNameVectorCEINMS() const
		{
			return vectDOFNameListCEINMS_;
		}

		inline const string& getModelName() const
		{
			return modelName_;
		}

		inline const Task& getTask ( int num ) const
		{
			return taskVect_[num];
		}

		inline int getTaskSize() const
		{
			return (int) taskVect_.size();
		}

		inline const double& getMin ( const string& DOFName ) const
		{
			return minDOF_.at ( DOFName );
		}

		inline const double& getMax ( const string& DOFName ) const
		{
			return maxDOF_.at ( DOFName );
		}

		inline const int& getnbOfStep() const
		{
			return nbOfStep_;
		}

		/**
		 * Write the muscle tendon length in the output directory.
		 */
		void writeLmt() const;

		/**
		 * Write the muscle moments arm in the output directory.
		 */
		void writeMa() const;

	protected:

		typedef boost::bimap < boost::bimaps::multiset_of<string>,
						boost::bimaps::multiset_of<string> > DOFMuscleMap;
		typedef DOFMuscleMap::value_type position;

		void setDOF ( const Task& task, bool betweenNode );
		void init ( const string& configurationFile );

		/**
		 * Return the max angles range for the coordinate.
		 * @param nameOfDOF OpenSim name of the DOF
		 */
		double getMinAnglesDof ( const string& nameOfDOF ) const;

		/**
		 * Return the min angles range for the coordinate.
		 * @param nameOfDOF OpenSim name of the DOF
		 */
		double getMaxAnglesDof ( const string& nameOfDOF ) const;

		/**
		 * Writes the angle in the storage in a .in file
		 */
		void writeAngles ( const Storage& storage, const string fileName );

		/**
		 * Writes the angle between node for test in a .in file and file an OpenSim storage.
		 */
		void writeAnglesBetweenNode ( const vector<vector<double> >& data, const string fileName, Storage& storage );

		void printMemUsage();
		int parseLine ( char* line );

		DOFMuscleMap DOFMuscleConnect_;
		Array<string> muscleNameListOpenSim_;
		Array<string> DOFNameListOpenSim_;
		vector<string> vectMuscleNameListCEINMS_;
		vector<string> vectDOFNameListCEINMS_;
		vector<string> vectMuscleNameListOpenSim_;
		vector<string> vectDOFNameListOpenSim_;
		vector<Task> taskVect_;
		Model osimModel_;
		string modelName_;
		string configurationFile_;
		SimTK::State si_;
		boost::shared_ptr<MuscleAnalysis> ptr_muscleAnalyse_;
		int nbOfStep_;
		bool printOsimData_;
		bool printCEINMSData_;
		map<string, double > minDOF_;
		map<string, double > maxDOF_;
		map<string, double> increments_;

};

#endif

#endif /* MUSCLEANALYSEFORSPLINE_H_ */
