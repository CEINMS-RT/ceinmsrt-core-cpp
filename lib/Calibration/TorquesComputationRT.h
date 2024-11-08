// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox".
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software,
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS.
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621,
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com

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
