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

#ifndef TorquesComputationRT_h
#define TorquesComputationRT_h

#include <vector>
#include <string>
#include <set>
#include "NMSmodel.h"
#include "TrialData.h"
#include "executionEMG.hxx"
#include "EMGPreProcessing.h"
#include <boost/unordered_map.hpp>
#include <time.h>
#include <algorithm> //for sorting only
#include <iostream>
using std::cout;
using std::endl;
#include "DataFromFile.h"
#include "MTUSpline.h"
#include "MTUSplineDataRead.h"
#include "TorquesComputation.h"

#include "ExecutionEmgXml.h"
#include <memory>

template<typename ComputationModeT, typename NMSmodelT>
class TorquesComputationRT;

template<typename ComputationModeT, typename NMSmodelT>
std::ostream& operator<< ( std::ostream& output, const TorquesComputationRT<ComputationModeT, NMSmodelT>& m );

template<typename ComputationModeT, typename NMSmodelT>
class TorquesComputationRT: public TorquesComputation<ComputationModeT, NMSmodelT>
{
	public:
		/**
		 * CONSTRUCTOR
		 **/
		TorquesComputationRT ( NMSmodelT& subject, const std::string& inputDataDirectory,
				const std::vector<std::string>& idTrials, const std::vector<std::string>& dofsToCalibrate,
				const std::string& emgFileName, const std::string& configurationFile, const std::string& subjectName,
				const std::string& translateName, const std::vector<double>& cropMin, const std::vector<double>& cropMax, const bool& filterEMG, const double& emd);

		void setTimeTorques ( std::vector<std::vector<double> >& TimeTorques );
		
		inline void setVerbose(const int& verbose)
		{
			_verbose = verbose;
		}

	protected:

		typedef std::map<std::string, std::vector<std::string> > MapSS;
		typedef std::vector<double> VectD;
		typedef boost::unordered_map<std::string, std::vector<std::string> > UnMapSS;

		void initEMG ( const int& i, const string& EMGDataFilename, bool useFilter = true );
		void initLmtMa ( const int& i, const string& AngleDataFilename );
		void computeLmtMafromSplines ( std::vector<std::shared_ptr<MTUSpline<1> > >& splines,
				const std::vector<double>& angles, std::vector<double>& lmt, std::vector<std::vector<double> >& ma );
		template<class T>
		void computeLmtMafromSplines ( T& splines, int dim, const std::vector<double>& angles, std::vector<double>& lmt,
				std::vector<std::vector<double> >& ma );
		int noMuscles_;
		std::auto_ptr<ExecutionEMGType> executionEMGPointer_;
		std::vector<MTUSplineDataRead::Task> taskVect_;
		std::vector<std::string> dofNameVect_;
		std::vector<std::string> muscleNamesVect_;
		std::vector<std::vector<std::string> > musclesNamesOnDof_;
		std::string configurationFile_;
		std::string subjectName_;
		std::vector<double> cropMin_;
		std::vector<double> cropMax_;
		std::vector<double> _maxEMG;
		bool _emgFirstPass;
		int _verbose;
		float emd_;
};

#include "TorquesComputationRT.cpp"

#endif
