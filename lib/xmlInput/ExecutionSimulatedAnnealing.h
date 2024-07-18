/*
 * Copyright (c) 2015, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
