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

#ifndef EXECUTIONSIMULATEDANNEALING_H
#define EXECUTIONSIMULATEDANNEALING_H

#include <CommonCEINMS.h>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <set>
#include "simulatedAnnealing.hxx"

class ExecutionSimulatedAnnealing
{
	public:
		
		enum CalibMode{
			ShapeFactor													= 1,
			StrengthCoefficients_ShapeFactor_TendonSlackLength_single 	= 2,
			GroupedStrengthCoefficients_IndividualLstLom				= 3,
			StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange = 4
		};
		
		
		ExecutionSimulatedAnnealing(const std::string& simulatedAnnealingFile);
		~ExecutionSimulatedAnnealing();
		
		inline const CalibMode& GetCalibMode() const
		{
			return calibMode_;
		}

		inline const std::string& getEMGProccesing() const
		{
			return _EMGProccesing;
		}

		inline const bool& getUseSpline() const
		{
			return _useSpline;
		}

		inline const std::string& getTranslateFileSpline() const
		{
			return _translateFileSpline;
		}

		inline const std::string& getTranslateFilePreScaling() const
		{
			return _translateFilePreScaling;
		}

		inline const std::string& getOsimFileSpline() const
		{
			return _scaledOsimFile;
		}

		inline const std::string& getOsimFilePreScaling() const
		{
			return _scaledOsimFile;
		}

		inline const int& getPrintOption() const
		{
			return _printOption;
		}

		inline const int& getNumberOfNode() const
		{
			return _numberOfNode;
		}

		inline const std::string& getUnscaledOsimFilePreScaling() const
		{
			return _unscaledOsimFile;
		}

		inline const std::string& getNameOfSubject() const
		{
			return _nameOfSubject;
		}

		inline const std::string& getTrialsDirectory() const
		{
			return _trialsDirectory;
		}

		inline const std::vector<std::string>& getTrialsName() const
		{
			return _trialsName;
		}

		inline const std::vector<std::string>& getDOFToCalibrate() const
		{
			return _DOFToCalibrate;
		}

		inline const std::vector<double>& getTrialscropMin() const
		{
			return _trialscropMin;
		}

		inline const std::vector<double>& getTrialscropMax() const
		{
			return _trialscropMax;
		}
		
		inline const bool& getUsePreScaling() const
		{
			return _usePreScaling;
		}
		
		inline const std::string& getSubjectXML() const
		{
			return _subjectXML;
		}
		
		inline const bool& getUseCalibration() const
		{
			return _useCalibration;
		}
		
		inline const bool& getFilterEMG() const
		{
			return _filterEMG;
		}
		
		inline const int& getMaxNoEval() const
		{
				return maxNoEval_;
		}
		
		inline const int& getNoEpsilon() const
		{
// 			COUT << "_noEpsilon " << _noEpsilon << std::endl;
				return _noEpsilon;
		}
		
		inline const double& getNT() const
		{
				return nt_;
		}
		
		inline const double& getNS() const
		{
				return ns_;
		}
		
		inline const double& getT() const
		{
				return t_;
		}
		
		inline const double& getRT() const
		{
				return rt_;
		}
		
		inline const double& getEpsilon() const
		{
				return _epsilon;
		}

		inline const double& getEMD() const
		{
			return emd_;
		}
		
		inline const std::vector<std::vector<std::string> >& getDOFSequenceToCalibrate() const
		{
			return _DOFSequenceToCalibrate;
		}

		inline const std::string& getInputSubjectXMLName() const
		{
			return inputSubjectXMLName_;
		}

		inline const std::string& getOutputSubjectXMLName() const
		{
			return outputSubjectXMLName_;
		}

	protected:

		std::auto_ptr<SimulatedAnnealingType> _annealingPointer;
		std::string _EMGProccesing;
		bool _useSpline;
		bool _usePreScaling;
		bool _useCalibration;
		bool _filterEMG;
		std::string _translateFileSpline;
		std::string _translateFilePreScaling;
		std::string _scaledOsimFile;
		int _printOption;
		int _numberOfNode;
		int maxNoEval_;
		int _noEpsilon;
		double nt_;
		double ns_;
		double rt_;
		double t_;
		double _epsilon;
		std::string _unscaledOsimFile;
		std::string _nameOfSubject;
		std::string _trialsDirectory;
		std::string _subjectXML;
		std::vector<std::string> _trialsName;
		std::vector<std::string> _DOFToCalibrate;
		std::vector<std::vector<std::string> > _DOFSequenceToCalibrate;
		std::vector<double> _trialscropMin;
		std::vector<double> _trialscropMax;
		std::string inputSubjectXMLName_;
		std::string outputSubjectXMLName_;

		CalibMode calibMode_;
		double emd_;
};
#endif // EXECUTIONSIMULATEDANNEALING_H
