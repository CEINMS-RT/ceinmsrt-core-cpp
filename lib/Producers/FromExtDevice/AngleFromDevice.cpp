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
#include "AngleFromDevice.h"
#include <iomanip>

// void AngleSigintHandler ( int sig )
// {
// 	SyncTools::Shared::endThreadMutex.lock();
// 	SyncTools::Shared::endThread = true;
// 	SyncTools::Shared::endThreadMutex.unlock();
// }

AngleFromDevice::AngleFromDevice(string xmlName, string executionName, bool record, std::string recordDirectory,
	std::string processDirectory, bool process) :
	firstPass_(true), xmlName_(xmlName), executionName_(executionName), _processDirectory(processDirectory),
	_process(process), _record(record), _recordDirectory(recordDirectory)
{
}

AngleFromDevice::~AngleFromDevice()
{
#ifdef VERBOSE

	// 	if ( _verbose > 1 )
	// 		COUT << "\033[1;31mAngle " << this << "\033[0m" << std::endl;

#endif
}

void AngleFromDevice::operator() ()
{
	// 	signal ( SIGINT, AngleSigintHandler );

	std::vector<double> idData;
	std::vector<double> angleData;
	std::string dynLib;
	bool useOfAngleAndComsumerPlugin = false;
	bool pluginBool = false;
	std::vector<double> angleDataPast;
	std::map<std::string, double> mapData;
	std::map<std::string, double> mapIdData;
	double time;

#ifdef TIMING
	timeval timeBeginWait, timeBegin, timeEnd;
	double timeDoubleBegin = 0, timeDoubleEnd = 0, timeDoubleTotal = 0, timeDoubleCurrent = 0;
	double timeDoubleBeginWait = 0, timeDoubleTotalWait = 0, timeDoubleCurrentWait = 0;
	int cptTotal = 0;
#endif

	// 	SyncTools::Shared::syncGuiMutex.lock();
	// 	_gui = SyncTools::Shared::syncGui;
	// 	SyncTools::Shared::syncGuiMutex.unlock();
	//
	// 	SyncTools::Shared::syncVerboseMutex.lock();
	// 	_verbose = SyncTools::Shared::syncVerbose;
	// 	SyncTools::Shared::syncVerboseMutex.unlock();

	getSyncGui(_gui);
	getSyncVerbose(_verbose);

	getDofNames(dofNames_);

	// 	SyncTools::Shared::lmtTimeMutex.lock();
	// 	SyncTools::Shared::lmtTime = 0;
	// 	SyncTools::Shared::lmtTimeMutex.unlock();

	ExecutionXmlReader xml(executionName_);

	if (!_process)
	{
		useOfAngleAndComsumerPlugin = xml.useOfAngleAndComsumerPlugin();
		pluginBool = xml.useOfAnglePlugin();
	}
	else
		pluginBool = true;

	if (useOfAngleAndComsumerPlugin)
	{
		dynLib = xml.getAngleAndComsumerPlugin();
		pluginBool = false;

		try
		{
			// 			COUT << xml.getAngleFile() << std::endl << std::flush;
			std::auto_ptr<ExecutionIKType> executionPointerIK(executionIK(xml.getAngleFile(), xml_schema::flags::dont_initialize));
			// 			COUT << executionPointerIK->OsimFile() << std::endl << std::flush;
			InterThread::setModelFileName(executionPointerIK->OsimFile());
		}
		catch (const xml_schema::exception& e)
		{
			COUT << e << endl;
			exit(EXIT_FAILURE);
		}
	}

	if (pluginBool)
	{
		if (!_process)
			dynLib = xml.getAnglePlugin();
		else
			dynLib = "libangle_file_plugins.so"; // Overrule the xml

		try
		{
			std::auto_ptr<ExecutionIKType> executionPointerIK(executionIK(xml.getAngleFile(), xml_schema::flags::dont_initialize));
			// 			SyncTools::Shared::modelFileName = executionPointerIK->OsimFile();
			// 			SyncTools::Shared::semModelFileName.notify();
			InterThread::setModelFileName(executionPointerIK->OsimFile());
		}
		catch (const xml_schema::exception& e)
		{
			COUT << e << endl;
			exit(EXIT_FAILURE);
		}
	}

	std::vector<std::string> dofNamePlugin;

	if (pluginBool)
	{
		setDynLib(dynLib);

		if (_record)
		{
			if (_process)
				plugin_->setDirectories(_recordDirectory, _processDirectory);
			else
				plugin_->setDirectories(_recordDirectory);
		}
		else
			if (_process)
				plugin_->setDirectories("output", _processDirectory);

		plugin_->setVerbose(_verbose);
		plugin_->setRecord(_record);

		plugin_->init(xmlName_, executionName_);
	}
	else if (useOfAngleAndComsumerPlugin)
	{
		_pluginAAC->setDofName(dofNames_);
		dofNamePlugin = _pluginAAC->GetDofName();
		for (std::vector<std::string>::const_iterator it = dofNamePlugin.begin(); it != dofNamePlugin.end(); it++)
			COUT << *it << std::endl << std::flush;
	}

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "Angle: waiting for ready to start..." << endl;

#endif

	double angleTime = -1;

	InterThread::readyToStart->wait();

	while (true)
	{
		// 		SyncTools::Shared::endThreadMutex.lock();

		if (InterThread::getEndThread())
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

		if (_verbose > 2)
			COUT << "Angle: waiting for consumer..." << endl;

#endif

		double lmtTime = InterThread::getLmtTime();

		if (firstPass_)
		{
			firstPass_ = false;
			// 			SyncTools::Shared::lmtTimeMutex.lock();
			// 			InterThread::setLmtTime(-1);
			// 			SyncTools::Shared::lmtTimeMutex.unlock();
		}
		else
			if (angleTime >= 0)
				InterThread::NMSconsumerDoneAngle.wait();
		// 			else
		// 				COUT << angleTime << std::endl;

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
		// 		double lmtTime = InterThread::getLmtTime();
		// 		SyncTools::Shared::lmtTimeMutex.unlock();

		if (emgTime >= lmtTime || angleTime == -1)
		{
			mapData.clear();
			mapIdData.clear();
			time = 0;
			idData.clear();
			angleData.clear();

			if (pluginBool)
			{
				mapData = plugin_->GetDataMap();
				time = plugin_->getTime();
				mapIdData = plugin_->GetDataMapTorque();

				// 		COUT << time << std::endl;
#ifdef VERBOSE

				if (_verbose > 2)
					COUT << "Angle time: " << plugin_->getTime() << std::endl;

#endif
			}
			else if (useOfAngleAndComsumerPlugin)
			{
				time = _pluginAAC->GetAngleTimeStamp();
				mapData = _pluginAAC->GetDataMap();
				mapIdData = _pluginAAC->GetDataMapTorque();

				// 				for ( vector<string>::const_iterator it1 = dofNamePlugin.begin(); it1 != dofNamePlugin.end(); it1++ )
				// 					mapData[*it1] = data.at ( std::distance<vector<string>::const_iterator> ( dofNamePlugin.begin(), it1 ) );
			}

			if (_gui)
			{
				// 				SyncTools::Shared::positionMapMutex.lock();
				// 				SyncTools::Shared::positionMap = mapData;
				// 				SyncTools::Shared::positionMapMutex.unlock();
				InterThread::setPositionMap(mapData);
			}

			for (vector<string>::const_iterator it1 = dofNames_.begin(); it1 != dofNames_.end(); it1++)
			{
				if (mapData.find(*it1) != mapData.end())
					angleData.push_back(mapData.at(*it1));
				else
				{
					COUT << "Joint " << *it1 << " not found in the IK result." << std::endl << std::flush;
					// 					angleData.push_back(0);
					// 					SyncTools::Shared::endThreadMutex.lock();
					// 					SyncTools::Shared::endThread = true;
					// 					SyncTools::Shared::endThreadMutex.unlock();
					InterThread::setEndThread(true);
				}

				if (mapIdData.find(*it1) != mapIdData.end())
					idData.push_back(mapIdData.at(*it1));
				else
					idData.push_back(0);
			}

			if (_gui)
			{
				// 				SyncTools::Shared::mutexGui.lock();
				// 				SyncTools::Shared::idGui.push_back ( idData );
				// 				SyncTools::Shared::mutexGui.unlock();
				InterThread::setGuiID(idData);
			}

			// 			COUT << angleData.front() << std::endl;
			InterThread::setExternalTorque(idData);
			updateAngle(angleData, time);
			angleTime = time;

			angleDataPast = angleData;

			// 			SyncTools::Shared::lmtTimeMutex.lock();
			// 			SyncTools::Shared::lmtTime = time;
			// 			SyncTools::Shared::lmtTimeMutex.unlock();
		}
		else
		{
			updateAngle(angleDataPast, angleTime);
			InterThread::setExternalTorque(idData);
			// 			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

#ifdef VERBOSE

		if (_verbose > 2)
			COUT << "Angle: waiting for Producer Done..." << endl;

#endif
		InterThread::angleProducingDone.notify();

#ifdef TIMING
		gettimeofday ( &timeEnd, NULL );
		timeDoubleEnd = ( timeEnd.tv_sec ) + 0.000001 * timeEnd.tv_usec;
		timeDoubleCurrent = timeDoubleEnd - timeDoubleBegin;
		timeDoubleTotal += timeDoubleCurrent;
#endif
	}

#ifdef TIMING
	// 	SyncTools::Shared::lmtTimeMutex.lock();
	COUT << "TimingAngle Mean: " << timeDoubleTotal / cptTotal * 1000 << " ms." << std::endl << std::flush;
	COUT << "TimingAngleWait Mean: " << timeDoubleTotalWait / cptTotal * 1000 << " ms." << std::endl << std::flush;
	// 	SyncTools::Shared::lmtTimeMutex.unlock();
#endif

	if (pluginBool)
	{
		plugin_->stop();
		closeDynLib();
	}

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "\033[1;32mAngle thread end\033[0m" << std::endl;

#endif
}