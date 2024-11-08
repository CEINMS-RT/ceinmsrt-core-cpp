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

#ifndef EMGPREPROCESSING_H_
#define EMGPREPROCESSING_H_



#include <vector>
#include <deque>
#include <cmath>
#include <iostream>
#include <ctime>
#include <iomanip> // for std::setprecision()
#ifdef UNIX
#include <sys/time.h>
#endif

#define NOMINMAX // Windows defines some macro's which conflict with functions/variables in the SimTK package. This statements prevents the definition of some macro's
#include <getTime.h>
#undef small	// See above. This statement undefines such a macro. See https://stackoverflow.com/questions/21165891/is-small-a-keyword-in-c for more info. min, max and small are used by SimTK.

/**
 * Class for EMG pre-processing.
 */

class EMGPreProcessing
{
public:

	/**
	 * Constructor
	 * @param aCoeffLP A coefficients for the low-pass filtering function transfer
	 * @param bCoeffLP B coefficients for the low-pass filtering function transfer
	 * @param aCoeffHP A coefficients for the high-pass filtering function transfer
	 * @param bCoeffHP B coefficients for the high-pass filtering function transfer
	 * @param maxAmp Maximum amplitude for the EMG for the normalization
	 */
	EMGPreProcessing(std::vector<double> aCoeffLP, std::vector<double> bCoeffLP, std::vector<double> aCoeffHP,
			std::vector<double> bCoeffHP, double maxAmp);

	/**
	 * Destructor
	 */
	virtual ~EMGPreProcessing();

	/**
	 * Realize the pre-processing on the data
	 * @param data The raw EMG
	 * @return The excitation
	 */
	double computeData(const double& data);

	/**
	 * Get the maximum EMG post-processing
	 */
	const double& getMax() const
	{
		return maxAmp_;
	}
	;

protected:
	std::vector<double> aCoeffLP_; //!< A coefficients for the low-pass filtering function transfer
	std::vector<double> bCoeffLP_; //!< B coefficients for the low-pass filtering function transfer
	std::vector<double> aCoeffHP_; //!< A coefficients for the high-pass filtering function transfer
	std::vector<double> bCoeffHP_; //!< B coefficients for the high-pass filtering function transfer
// 	std::vector<double> aCoeffDC_; //!< A coefficients for the high-pass filtering function transfer for cutting the DC components
// 	std::vector<double> bCoeffDC_; //!< B coefficients for the high-pass filtering function transfer for cutting the DC components
	double maxAmp_; //!< Maximum amplitude for the EMG for the normalization post-processing
	double maxAmpRaw_; //!< Maximum amplitude for the EMG for the normalization pre-processing
	std::deque<double> pastDataHP_; //!< Past data for function transfer high-pass
	std::deque<double> pastDataFilterHP_; //!< Past data for function transfer high-pass
	std::deque<double> pastDataLP_; //!< Past data for function transfer low-pass
	std::deque<double> pastDataFilterLP_; //!< Past data for function transfer low-pass
// 	std::deque<double> pastDataDC_; //!< Past data for function transfer high-pass for cutting the DC components
// 	std::deque<double> pastDataFilterDC_; //!< Past data for function transfer high-pass for cutting the DC components
	double timeInit_;
	double timePrevious_;
	bool timeBool_, timeOKBool_; 
	bool calibration_; //
};

#endif /* EMGPREPROCESSING_H_ */
