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


#include "EMGFromDevice.h"
#ifdef UNIX
#include <dlfcn.h>
#endif

// void EMGSigintHandler ( int sig )
// {
// 	SyncTools::Shared::endThreadMutex.lock();
// 	SyncTools::Shared::endThread = true;
// 	SyncTools::Shared::endThreadMutex.unlock();
// }

EMGFromDevice::EMGFromDevice ( std::string xmlName, std::string executionName, bool record, std::string recordDirectory,
	std::string processDirectory, bool process ) :
	firstPass_ ( true ), xmlName_ ( xmlName ), execName_ ( executionName ), _processDirectory ( processDirectory ),
	_process ( process ), _record ( record ), _recordDirectory ( recordDirectory )
{

}

EMGFromDevice::~EMGFromDevice()
{
#ifdef VERBOSE

	if ( _verbose > 1 )
		COUT << "\033[1;31mEMG " << this << "\033[0m" << std::endl;

#endif
}

void EMGFromDevice::computeChannelNameOnMuscle()
{
// 	using namespace SyncTools::Shared;
	MapStrVectStr channelNameOnMuscleTemp;

	// Create a map between muscle and channel.
	for ( MapStrVectStr::const_iterator it1 = musclesNamesOnChannel_.begin(); it1 != musclesNamesOnChannel_.end(); it1++ )
		for ( std::vector<std::string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++ )
			channelNameOnMuscleTemp[*it2].push_back ( it1->first );


// 	SyncTools::Shared::musclesNamesMutex.lock();
// 	muscleName_ = SyncTools::Shared::musclesNames;
// 	SyncTools::Shared::musclesNamesMutex.unlock();
		
	getMusclesNames(muscleName_);

	if ( muscleName_.size() == 0 )
	{
		for ( MapStrVectStr::const_iterator it1 = channelNameOnMuscleTemp.begin(); it1 != channelNameOnMuscleTemp.end();
				it1++ )
			muscleName_.push_back ( it1->first );
	}

	setMusclesNames(muscleName_);
	
// 	SyncTools::Shared::musclesNamesMutex.lock();
// 	SyncTools::Shared::musclesNames = muscleName_;
// 	SyncTools::Shared::musclesNamesMutex.unlock();

	// Use musclesNames and unordered boost map for having the same correspondence between the emg and the muscle.
	for ( std::vector<std::string>::const_iterator it1 = muscleName_.begin(); it1 != muscleName_.end(); it1++ )
	{
		try
		{
			channelNameOnMuscle_[*it1] = channelNameOnMuscleTemp.at ( *it1 );
		}
		catch ( const std::out_of_range& oor )
		{
#ifdef VERBOSE

			if ( _verbose > 1 )
				cout << *it1 << " will be put to zero because it was not found in the channel" << endl;

#endif
		}
	}
}

// void EMGFromDevice::getMusclesNamesOnChannel()
// {
// 	SyncTools::Shared::musclesNamesOnChannelSem.wait();
// 	musclesNamesOnChannel_ = SyncTools::Shared::musclesNamesOnChannel;
// 	SyncTools::Shared::musclesNamesOnChannelSem.notify();
// }

void EMGFromDevice::operator() ()
{

// 	SyncTools::Shared::syncVerboseMutex.lock();
// 	_verbose = SyncTools::Shared::syncVerbose;
// 	SyncTools::Shared::syncVerboseMutex.unlock();
	getSyncVerbose(_verbose);

// 	signal ( SIGINT, EMGSigintHandler );

	std::string dynLib;
	bool pluginBool;
	std::vector<double> EMGDataPast;

#ifdef TIMING
	timeval timeBeginWait, timeBegin, timeEnd;
	double timeDoubleBegin = 0, timeDoubleEnd = 0, timeDoubleTotal = 0, timeDoubleCurrent = 0;
	double timeDoubleBeginWait = 0, timeDoubleTotalWait = 0, timeDoubleCurrentWait = 0;
	int cptTotal = 0;
#endif

	ExecutionXmlReader xml ( execName_ );

	if ( !_process )
	{
		pluginBool = xml.useOfAnglePlugin();
		dynLib = xml.getEmgPlugin();
	}
	else
	{
		pluginBool = true;
		dynLib = "libemg_file_plugins.so"; // Overrule the xml
	}

	setDynLib ( dynLib );

	if ( _record )
	{
		if ( _process )
			plugin_->setDirectories ( _recordDirectory, _processDirectory );
		else
			plugin_->setDirectories ( _recordDirectory );

	}
	else
		if ( _process )
			plugin_->setDirectories ( "output", _processDirectory );

	plugin_->setVerbose ( _verbose );
	plugin_->setRecord ( _record );

	plugin_->init ( xmlName_, execName_ );

#ifdef VERBOSE

	if ( _verbose > 1 )
		COUT << "EMG: waiting for muscle on Channel..." << endl;

#endif

	getMusclesNamesOnChannel(musclesNamesOnChannel_);
	computeChannelNameOnMuscle();



#ifdef VERBOSE

	if ( _verbose > 1 )
		cout << "EMG: waiting for ready to start..." << endl;

#endif


// 	SyncTools::Shared::EMGTimeMutex.lock();
// 	SyncTools::Shared::EMGTime = -1;
// 	SyncTools::Shared::EMGTimeMutex.unlock();

 	InterThread::readyToStart->wait();

	while ( true )
	{
// 		SyncTools::Shared::endThreadMutex.lock();

		if ( InterThread::getEndThread() )
		{
// 			SyncTools::Shared::endThreadMutex.unlock();
			InterThread::notifyAll();
// 			COUT << "stop" << std::endl << std::flush;
			break;
		}

// 		SyncTools::Shared::endThreadMutex.unlock();

#ifdef TIMING
		gettimeofday ( &timeBeginWait, NULL );
		timeDoubleBeginWait = ( timeBeginWait.tv_sec ) + 0.000001 * timeBeginWait.tv_usec;
#endif

#ifdef VERBOSE

		if ( _verbose > 2 )
			COUT << "EMG: waiting for consumer..." << endl;

#endif

		if ( firstPass_ )
		{
			firstPass_ = false;
			InterThread::setEMGTime(-1);
		}
		else
			InterThread::NMSconsumerDoneEMG.wait();

#ifdef TIMING
		gettimeofday ( &timeBegin, NULL );
		timeDoubleBegin = ( timeBegin.tv_sec ) + 0.000001 * timeBegin.tv_usec;
		timeDoubleCurrentWait = timeDoubleBegin - timeDoubleBeginWait;
		timeDoubleTotalWait += timeDoubleCurrentWait;
		cptTotal ++;
#endif

// 		SyncTools::Shared::EMGTimeMutex.lock();
		double emgTime = InterThread::getEMGTime();
// 		SyncTools::Shared::EMGTimeMutex.unlock();

// 		SyncTools::Shared::lmtTimeMutex.lock();
		double lmtTime = InterThread::getLmtTime();
// 		SyncTools::Shared::lmtTimeMutex.unlock();

		map<string, double> mapMuscle;

		vector<double> EMGData;
		const map<string, double>& mapData = plugin_->GetDataMap();

		for ( UnMapStrVectStr::const_iterator it1 = channelNameOnMuscle_.begin(); it1 != channelNameOnMuscle_.end();
				it1++ )
		{
			double data = 0;

			if ( it1->second.size() != 0 )
			{

				for ( vector<string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++ )
				{
					try
					{
						data += mapData.at ( *it2 );
					}
					catch ( const std::out_of_range& oor )
					{
						COUT << "Channel " << *it2 << "not found in the EMG file." << std::endl;

// 						SyncTools::Shared::endThreadMutex.lock();
// 						SyncTools::Shared::endThread = true;
// 						SyncTools::Shared::endThreadMutex.unlock();
						InterThread::setEndThread(true);
						break;
					}

				}

				data /= it1->second.size();
			}

			mapMuscle[it1->first] = data;
		}

		for ( vector<string>::const_iterator it1 = muscleName_.begin(); it1 != muscleName_.end(); it1++ )
		{
			if ( mapMuscle.find ( *it1 ) != mapMuscle.end() )
				EMGData.push_back ( mapMuscle.at ( *it1 ) );
			else
				EMGData.push_back ( 0 );
		}

#ifdef TIMING
		gettimeofday ( &timeEnd, NULL );
		timeDoubleEnd = ( timeEnd.tv_sec ) + 0.000001 * timeEnd.tv_usec;
		timeDoubleCurrent = timeDoubleEnd - timeDoubleBegin;
		timeDoubleTotal += timeDoubleCurrent;
#endif

		updateEmg ( EMGData, plugin_->getTime() );
// 		SyncTools::Shared::EMGTimeMutex.lock();
// 		SyncTools::Shared::EMGTime = plugin_->getTime();
		InterThread::setEMGTime(plugin_->getTime());
// 		SyncTools::Shared::EMGTimeMutex.unlock();
		EMGDataPast = EMGData;


#ifdef VERBOSE

		if ( _verbose > 2 )
			COUT << "EMG: waiting for Producer..." << endl;

#endif

		InterThread::emgProducingDone.notify();
	}

#ifdef TIMING
// 	SyncTools::Shared::lmtTimeMutex.lock();
	COUT << "TimingEMG Mean: " << timeDoubleTotal / cptTotal * 1000 << " ms." << std::endl << std::flush;
	COUT << "TimingEMGWait Mean: " << timeDoubleTotalWait / cptTotal * 1000 << " ms." << std::endl << std::flush;
// 	SyncTools::Shared::lmtTimeMutex.unlock();
#endif

	plugin_->stop();
	closeDynLib();

#ifdef VERBOSE

	if ( _verbose > 1 )
		COUT << "\033[1;32mEMG thread end\033[0m" << std::endl;

#endif
}
