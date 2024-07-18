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

#include "EMGPreProcessing.h"

EMGPreProcessing::EMGPreProcessing(std::vector<double> aCoeffLP, std::vector<double> bCoeffLP,
		std::vector<double> aCoeffHP, std::vector<double> bCoeffHP, double maxAmp) :
		aCoeffLP_(aCoeffLP), bCoeffLP_(bCoeffLP), aCoeffHP_(aCoeffHP), bCoeffHP_(bCoeffHP), maxAmp_(maxAmp), maxAmpRaw_(0), timeBool_(true), timeOKBool_(false), timePrevious_(-1), calibration_(true)
{

	// Fill the past data for the function transfer to zero
	for (std::vector<double>::iterator it = bCoeffHP_.begin(); it != bCoeffHP_.end(); it++)
	{
		pastDataHP_.push_back(0);
		pastDataFilterHP_.push_back(0);
	}
	for (std::vector<double>::iterator it = bCoeffLP_.begin(); it != bCoeffLP_.end(); it++)
	{
		pastDataLP_.push_back(0);
		pastDataFilterLP_.push_back(0);
	}
// 	for (std::vector<double>::iterator it = bCoeffDC_.begin(); it != bCoeffDC_.end(); it++)
// 	{
// 		pastDataDC_.push_back(0);
// 		pastDataFilterDC_.push_back(0);
// 	}
	pastDataFilterLP_.pop_back();
	pastDataFilterHP_.pop_back();/*
	pastDataFilterDC_.pop_back();*/
}

EMGPreProcessing::~EMGPreProcessing()
{

}

double EMGPreProcessing::computeData(const double& data)
{
	if (timeBool_)
	{
#ifdef WIN32
		timeInit_ = rtb::getTime();
#endif
#ifdef UNIX
		timeval tv;
		gettimeofday(&tv, NULL);
		timeInit_ = (tv.tv_sec) + (0.000001 * tv.tv_usec);
#endif
		timeBool_ = false;
		std::cout << "EMG calibration started" << std::endl;
	}


	double dataOut = 0;

	//if(data > maxAmpRaw_) maxAmpRaw_ = data;

	pastDataHP_.push_front(data);
	pastDataHP_.pop_back();

	// Low pass filtering
	for (std::vector<double>::const_iterator it = bCoeffHP_.begin(); it != bCoeffHP_.end(); it++)
		dataOut += *it * pastDataHP_[std::distance<std::vector<double>::const_iterator>(bCoeffHP_.begin(), it)];
	for (std::vector<double>::const_iterator it = aCoeffHP_.begin(); it != aCoeffHP_.end(); it++)
		dataOut -= *it * pastDataFilterHP_[std::distance<std::vector<double>::const_iterator>(aCoeffHP_.begin(), it)];

//	std::cout << bCoeffHP_[0] << std::endl;

	pastDataFilterHP_.push_front(dataOut);
	pastDataFilterHP_.pop_back();

//	std::cout << dataOut << std::endl;
	// Rectification
	dataOut = std::abs(dataOut);
//	std::cout << dataOut << std::endl;
//
	pastDataLP_.push_front(dataOut);
	pastDataLP_.pop_back();

	dataOut = 0;

	// high-pass filtering
	for (std::vector<double>::const_iterator it = bCoeffLP_.begin(); it != bCoeffLP_.end(); it++)
		dataOut += *it * pastDataLP_[std::distance<std::vector<double>::const_iterator>(bCoeffLP_.begin(), it)];
	for (std::vector<double>::const_iterator it = aCoeffLP_.begin(); it != aCoeffLP_.end(); it++)
		dataOut -= *it * pastDataFilterLP_[std::distance<std::vector<double>::const_iterator>(aCoeffLP_.begin(), it)];

	pastDataFilterLP_.push_front(dataOut);
	pastDataFilterLP_.pop_back();

//	pastDataDC_.push_front(dataOut);
//	pastDataDC_.pop_back();

//	// high-pass filtering for cutting the DC composant
//	for (std::vector<double>::const_iterator it = bCoeffDC_.begin(); it != bCoeffDC_.end(); it++)
//		dataOut += *it * pastDataDC_[std::distance<std::vector<double>::const_iterator>(bCoeffDC_.begin(), it)];
//	for (std::vector<double>::const_iterator it = aCoeffDC_.begin(); it != aCoeffDC_.end(); it++)
//		dataOut -= *it * pastDataFilterDC_[std::distance<std::vector<double>::const_iterator>(aCoeffDC_.begin(), it)];
//
////	std::cout << bCoeffHP_[0] << std::endl;
//
//	pastDataFilterDC_.push_front(dataOut);
//	pastDataFilterDC_.pop_back();

	// Get the max
	/*if (timeOKBool_)
	{
		double timeNow;
#ifdef WIN32
		getTime(timeNow);
#endif
#ifdef UNIX
		timeval tv;
		gettimeofday(&tv, NULL);
		timeNow = (tv.tv_sec) + (0.000001 * tv.tv_usec);
#endif
		if (timeNow - timeInit_ > 5)
			timeOKBool_ = false;
		dataOut = 0;
	}*/

	if (calibration_)
	{
		double timeNow;
		timeNow = rtb::getTime();

		if (timeNow - timeInit_ > 10) // Calibrate for 10 seconds (don't show if EMG_max increased)
		{
			std::cout << "EMG calibration done" << std::endl;
			calibration_ = false;
		}
	}

	if (dataOut > maxAmp_ && !timeOKBool_) // Continuously update maxAmp_
	{
		double timeNow;
		timeNow = rtb::getTime();

		if (!calibration_ && timeNow - timePrevious_ > 1) // Show if EMG max is increase (only once per second and not during calibration to prevent cluttering output window)
		{
			std::cout << "Time: " << timeNow - timeInit_ << " | EMG_max of a muscle increased" << std::endl;
			timePrevious_ = timeNow;
		}

		maxAmp_ = dataOut;
	}
	
	// Normalization
	if (maxAmp_ != 0)
		dataOut /= maxAmp_;

	// Limit between 1 and 0
	if (dataOut > 1)
		dataOut = 1;
	else if (dataOut < 0)
		dataOut = 0;
	
//	dataOut *= maxAmpRaw_;


	return dataOut;
}
