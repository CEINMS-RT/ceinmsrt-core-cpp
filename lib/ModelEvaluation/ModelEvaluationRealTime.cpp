// #include "ModelEvaluationRealTime.h"

template<typename NMSmodelT>
ModelEvaluationRealTime<NMSmodelT>::ModelEvaluationRealTime(std::string xmlName, NMSmodelT& subject, std::string executionName, bool record, std::string recordDirectory,
	std::string processDirectory, bool process) :
	subject_(subject), cpt_(0), _executionName(executionName), _record(record), _recordDirectory(recordDirectory),
	_processDirectory(processDirectory), _process(process), xmlName_(xmlName), _verbose(0), _useOfEmgAndAngleAndComsumerPlugin(false)
{
	//### CHANGE FOR PROSTHESIS CONTROL ###
	//COUT << "This Version have been change for Prosthesic control (Add muscle torque and EMG)" << std::endl;
	//### CHANGE FOR PROSTHESIS CONTROL ###

}

template<typename NMSmodelT>
ModelEvaluationRealTime<NMSmodelT>::~ModelEvaluationRealTime()
{
#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "\033[1;31mNMS " << this << "\033[0m" << std::endl;

#endif

	for (std::vector<FilterKin::AvrFilt<double>* >::iterator it = _torqueFilter.begin(); it != _torqueFilter.end(); it++)
		delete* it;
}

template<typename NMSmodelT>
void ModelEvaluationRealTime<NMSmodelT>::operator() ()
{
	std::vector<double> emgFromQueue;
	std::vector<double> optimalFiberLength;
	std::vector<double> isometricsMaxForce;
	std::vector<double> lmtFromQueue;
	std::vector<double> dataTorque, muscleForce, muscleForceActive, muscleForcePassive, dataExternalTorque, fiberLength, fiberVelocity, tendonStrain;
	std::vector<double> dataTorqueCpy;
	std::vector<std::vector<double> > momentArmsFromQueue;
	std::map<std::string, std::vector<std::string> > musclesNamesOnChannel;

#ifdef UNIX
	timeval now;
	timeval timeConsumeStruct;
#endif

	double timeNow;
	double timeNowPast = rtb::getTime();
	double timeConsume;
	double emgTime;
	double lmtMaTime = -1;
	double emgTimePast = -1;
	double lmtMaTimePast = -1;

	std::string dynLib;
	std::string dynLibOptimization;

	int port;

#ifdef TIMING
	timeval timeBeginWait;
	timeval timeBegin;
	timeval timeEnd;
	double timeDoubleBegin = 0;
	double timeDoubleEnd = 0;
	double timeDoubleTotal = 0;
	double timeDoubleCurrent = 0;
	double timeDoubleBeginWait = 0;
	double timeDoubleTotalWait = 0;
	double timeDoubleCurrentWait = 0;
	int cptTotal = 0;
#endif

	subject_.getDoFNames(dofNames_); // Get Dof name from model

	setDofNamesToShared(dofNames_); // Share DOF name to other thread
	noDof_ = dofNames_.size();
	momentArmsFromQueue.resize(noDof_);

	subject_.getMuscleNames(muscleNames_);
	setMuscleNamesToShared(muscleNames_);

	subject_.getMusclesNamesOnChannel(musclesNamesOnChannel);
	setMusclesNamesOnChannel(musclesNamesOnChannel);

	for (int cpt = 0; cpt < noDof_; cpt++)
		_torqueFilter.push_back(new FilterKin::AvrFilt<double>(5));

	getSyncGui(_gui); // Get if we use GUI
	getSyncVerbose(_verbose); // Get the level of verbose

	subject_.getOptimalFiberLengths(optimalFiberLength);
	InterThread::setOptimaFiberLength(optimalFiberLength);

	subject_.getstrengthCoefficientWithMaxIsometrics(isometricsMaxForce);
	InterThread::setIsometricsMaxForce(isometricsMaxForce);

	ExecutionXmlReader xml(_executionName);

	bool useOfAngleAndComsumerPlugin = false;
	if (!_process)
	{
		useOfAngleAndComsumerPlugin = xml.useOfAngleAndComsumerPlugin(); // // if we use external plugin that need EMG-driven torque that also give the angle DOF
		if (xml.useOfEmgAndAngleAndComsumerPlugin())
		{
			_useOfEmgAndAngleAndComsumerPlugin = true;
			useOfAngleAndComsumerPlugin = false;
		}
	}

	initEMGPlugin(); // Initilaisation of the EMG external plugin


#ifndef NO_SAVE
	OpenSimFileLogger<NMSmodelT>* logger;

	if (_record) // Initialisation of the logger for recording on file
	{
		logger = new OpenSimFileLogger<NMSmodelT>(subject_, _recordDirectory);
		logger->addLog(Logger::Activations, muscleNames_);
		logger->addLog(Logger::FibreLengths, muscleNames_);
		logger->addLog(Logger::FibreVelocities, muscleNames_);
		logger->addLog(Logger::MuscleForces, muscleNames_);
		logger->addLog(Logger::LMT, muscleNames_);
		logger->addLog(Logger::Torques, dofNames_);
		//logger->addLog(Logger::TorquesFilt, dofNames_);   // do not use anymore
		logger->addLog(Logger::PennationAngle, muscleNames_);
		logger->addLog(Logger::TendonLength, muscleNames_);
		logger->addLog(Logger::MusclePassiveForces, muscleNames_);
		logger->addLog(Logger::MuscleActiveForces, muscleNames_);
		logger->addLog(Logger::TendonStrain, muscleNames_);
		logger->addLog(Logger::NMSTiming);
		logger->addLog(Logger::TotalTiming);
		logger->addLog(Logger::Emgs, muscleNames_);
	}

#endif

	bool pluginBool = xml.useOfComsumerPlugin(); // if we use external plugin that need EMG-driven torque

	bool pluginOptimizationBool = xml.useOfOptimizationPlugin();

	if (useOfAngleAndComsumerPlugin)
	{
		dynLib = xml.getAngleAndComsumerPlugin();
		pluginBool = false;
	}
	else if (_useOfEmgAndAngleAndComsumerPlugin)
	{
		dynLib = xml.getEmgAndAngleAndComsumerPlugin();
		pluginBool = false;
	}

	if (pluginOptimizationBool)
	{
		std::cout << "xml.getOptimizationPlugin: " << xml.getOptimizationPlugin() << std::endl;
		dynLibOptimization = xml.getOptimizationPlugin();
		_optimizationPlugin.setDynLib(dynLibOptimization);

		if (_record) // give the directory for the recoding on file an dalso the directory where the file we read from are if needed
		{
			if (_process)
				_optimizationPlugin.getPlugin()->setDirectories(_recordDirectory, _processDirectory);
			else
				_optimizationPlugin.getPlugin()->setDirectories(_recordDirectory);

		}
		else if (_process)
			_optimizationPlugin.getPlugin()->setDirectories("output", _processDirectory);

		_optimizationPlugin.getPlugin()->setVerbose(_verbose); // set the level of verbose for the plugin
		_optimizationPlugin.getPlugin()->setRecord(_record); // set if we record data on file

		//_optimizationPlugin.getPlugin()->init(subject_, _executionName);
		_optimizationPlugin.getPlugin()->init(subject_, _executionName, xmlName_);
	}

	if (pluginBool)
	{
#ifdef VERBOSE

		if (_verbose > 1)
			COUT << "Load Comsumer plugin" << std::endl;

#endif
		dynLib = xml.getComsumerPlugin();
		port = atoi(xml.getComsumerPort().c_str());
	}

	if (pluginBool)
	{
#ifdef VERBOSE

		if (_verbose > 1)
			COUT << "init Comsumer plugin" << std::endl;

#endif

		setDynLib(dynLib);
		plugin_->init(port); // Initialisation of the plugin

		plugin_->setMuscleName(muscleNames_); // set muscle name
		plugin_->setDofName(dofNames_); // set dof name
	}
	else if (useOfAngleAndComsumerPlugin)
	{
		//Moved to LmtMaFromMTUSpline
		//_pluginAAC->setMuscleName(muscleNames_);
		//_pluginAAC->setDofName(dofNames_);// set dof name
	}
	else if (_useOfEmgAndAngleAndComsumerPlugin)
	{
		_pluginEAC->setMuscleName(muscleNames_);
		_pluginEAC->setDofName(dofNames_);// set dof name
	}

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "NMS: waiting for ready To Start ..." << std::endl;

#endif

	InterThread::readyToStart->wait(); // Wait for GUI and LmtandMaFromMTUSpline

/*    if ( _verbose > 0 )
		std::cout << "\033[0;32mBegin the Real-time NMS computation\033[0m" << std::endl;
*/
	getMusclesNamesOnDofsFromShared(muscleNamesOnDof_); // get the name of muscle on the DOF


	if (_record) // if we record on file initilaise the logger for MA
		for (std::vector<std::string>::const_iterator it = dofNames_.begin(); it != dofNames_.end(); it++)
		{
			const unsigned short& cpt = std::distance<std::vector<std::string>::const_iterator>(dofNames_.begin(), it);
			logger->addMa(*it, muscleNamesOnDof_.at(cpt));
		}


	if (pluginOptimizationBool)
		_optimizationPlugin.getPlugin()->start();
	bool firstPass = true; // Temporary to optimize all data points

	while (true)
	{

		if (InterThread::getEndThread()) // Stop condition
		{
			InterThread::notifyAll();
			break;
		}

#ifdef TIMING
		timeDoubleBeginWait = rtb::getTime();
#endif

		//WaitForProducer(); // wait for the other thread to produce data (Lmt and ma thread)
		timeConsume = rtb::getTime();
#ifdef TIMING
		timeDoubleBegin = rtb::getTime();
		timeDoubleCurrentWait = timeDoubleBegin - timeDoubleBeginWait;
		timeDoubleTotalWait += timeDoubleCurrentWait;
		cptTotal++;
#endif






#ifdef VERBOSE

		if (_verbose > 2)
			std::cout << "NMS: getting data MA ..." << std::endl;

#endif

		getEMG(emgFromQueue, emgTime); // get EMG data and time of recording

		if (firstPass)
		{
			lmtMaTime = InterThread::getLmtTime();
			firstPass = false;
		}
		while (!(emgTime <= lmtMaTime || !_process) && !InterThread::getEndThread()) // if the EMG are before the IK 
		{
			lmtMaTime = InterThread::getLmtTime(); // get the time of recording of Lmt and Ma
			if (lmtMaTime != -1 && !InterThread::isEmptyLMT()) // if the time of recording is -1 is mean that no data have been captured yet
			{
				getLmtFromShared(lmtFromQueue); // Get Lmt
				InterThread::NMSconsumerDoneAngle.notify();
				lmtFromQueue.pop_back();

				for (unsigned int i = 0; i < noDof_; ++i) // get MA
				{
					getMomentArmsFromShared((momentArmsFromQueue.at(i)), i);
					momentArmsFromQueue.at(i).pop_back(); //removes time value from the end of vector
				}
			}

		}

		if (!_process)
		{
			lmtMaTime = InterThread::getLmtTime();
			getLmtFromShared(lmtFromQueue); // Get Lmt
			InterThread::NMSconsumerDoneAngle.notify();
			lmtFromQueue.pop_back();
			for (unsigned int i = 0; i < noDof_; ++i) // get MA
			{
				getMomentArmsFromShared((momentArmsFromQueue.at(i)), i);
				momentArmsFromQueue.at(i).pop_back(); //removes time value from the end of vector
			}
		}

		timeConsume = rtb::getTime();

#ifdef VERBOSE

		//if ( _verbose > 2 )
		//COUT << "NMS: getting data EMG ..." << std::endl;

#endif


#ifdef VERBOSE

		if (_verbose > 2)
			COUT << "NMS: Computing ..." << std::endl;

#endif

		timeNow = rtb::getTime();

		if (emgTimePast < emgTime && emgTime >= 0) // if we have new EMG data and time is <= 0 (data availible)
		{
			if (!emgFromQueue.empty())
			{
				if (pluginOptimizationBool)
				{
					_optimizationPlugin.getPlugin()->setTime(emgTime);
					_optimizationPlugin.getPlugin()->setEmgs(emgFromQueue);
				}

				subject_.setEmgs(emgFromQueue); // Update EMG for the model
				subject_.updateActivations(); // conpute activation
				subject_.pushState();
#ifndef NO_SAVE

				if (_record) // save to file
				{
					logger->log(Logger::Emgs, emgTime);
					logger->log(Logger::Activations, emgTime);
					logger->log(Logger::TendonLength, emgTime);
				}

#endif
#ifdef USE_GUI

				if (_gui) // send data to the GUI
				{
					timeNow = rtb::getTime();
					InterThread::setGuiEMG(emgFromQueue, emgTime);
					//InterThread::setGuiTimeEmg ( emgTime );
					InterThread::AddToTimeEndNMS(timeNow - emgTime);
					InterThread::AddToNbFrameEMG(1);
				}

#endif

				emgTimePast = emgTime;
			}
		}

		{
			if (lmtMaTimePast < lmtMaTime && lmtMaTime >= 0) // if we have new data and valide data
			{
				if (!lmtFromQueue.empty() && !momentArmsFromQueue.empty()) // if we have lmt and Ma data
				{
					getExternalTorqueFromShared(dataExternalTorque);
					if (pluginOptimizationBool)
					{
						_optimizationPlugin.getPlugin()->setExternalDOFTorque(dataExternalTorque);
						_optimizationPlugin.getPlugin()->setLmt(lmtFromQueue);
						_optimizationPlugin.getPlugin()->setMA(momentArmsFromQueue);

						_optimizationPlugin.getPlugin()->newData();
					}

					subject_.setTime(lmtMaTime); // Update time for the model
					subject_.setMuscleTendonLengths(lmtFromQueue); // Update LMt for the model


					for (unsigned int i = 0; i < noDof_; ++i)
						subject_.setMomentArms(momentArmsFromQueue.at(i), i); // update MA for the model

					subject_.updateState(); // Compute the torque 
					subject_.pushState();
					subject_.getNormFiberVelocities(fiberVelocity);
					subject_.getNormFiberLengths(fiberLength);
					subject_.getMuscleActiveForces(muscleForceActive); // get muscle force
					subject_.getMusclePassiveForces(muscleForcePassive); // get muscle force
					subject_.getTendonStrain(tendonStrain);
					subject_.getTorques(dataTorque); // get the torque 
					subject_.getMuscleForces(muscleForce); // get muscle force

#ifndef NO_SAVE
					if (_record) // save to file
					{
						timeNow = rtb::getTime();
						double timing = timeNow - timeConsume;
						logger->log(Logger::MuscleForces, lmtMaTime);
						logger->log(Logger::MusclePassiveForces, lmtMaTime);
						logger->log(Logger::MuscleActiveForces, lmtMaTime);
						logger->log(Logger::TendonStrain, lmtMaTime);
						logger->log(Logger::Torques, lmtMaTime);
						// logger->log(Logger::TorquesFilt, lmtMaTime);  // do not use anymore
						logger->log(Logger::LMT, lmtMaTime);
						logger->log(Logger::NMSTiming, lmtMaTime, timing);
						logger->log(Logger::FibreLengths, lmtMaTime);
						logger->log(Logger::FibreVelocities, lmtMaTime);
						logger->log(Logger::PennationAngle, lmtMaTime);
						logger->logMa(dofNames_, lmtMaTime, momentArmsFromQueue);
						timing = timeNow - timeNowPast;
						timeNowPast = timeNow;
						logger->log(Logger::TotalTiming, lmtMaTime, timing);
					}
#endif

					// send the computed MTForces to the plugin
					if (pluginOptimizationBool) {
						_optimizationPlugin.getPlugin()->setMuscleForce(muscleForce);
					}

					if (pluginBool) // if we use a consumer plugin
					{
#ifdef VERBOSE
						if (!useOfAngleAndComsumerPlugin)
						{
							if (_verbose > 2)
								COUT << "Send Data Comsumer plugin" << std::endl;

#endif
							plugin_->setTimeStamp(lmtMaTime);
							plugin_->setMuscleForce(muscleForce);
							plugin_->setDofTorque(dataTorque);
							plugin_->spin();
						}
					}
					else if (useOfAngleAndComsumerPlugin)
					{
						_pluginAAC->setOutputTimeStamp(lmtMaTime);
						_pluginAAC->setMuscleFiberVelocity(fiberVelocity);
						_pluginAAC->setMuscleFiberLength(fiberLength);
						_pluginAAC->setMuscleForcePassive(muscleForcePassive);
						_pluginAAC->setMuscleForceActive(muscleForceActive);
						_pluginAAC->setTendonStrain(tendonStrain);
						_pluginAAC->setDofTorque(dataTorque);
						_pluginAAC->setMuscleForce(muscleForce);
					}
					else if (_useOfEmgAndAngleAndComsumerPlugin)
					{
						_pluginEAC->setOutputTimeStamp(lmtMaTime);
						_pluginEAC->setDofTorque(dataTorque);
						_pluginEAC->setMuscleForce(muscleForce);
						_pluginEAC->setMuscleFiberVelocity(fiberVelocity);
						_pluginEAC->setMuscleFiberLength(fiberLength);
					}

#ifdef USE_GUI



					if (_gui) // update GUI
					{

						std::vector<double> data;
						subject_.getFiberLengths(data);
						InterThread::setGuiTorque(dataTorque);
						InterThread::setGuiMuscleForce(muscleForce);
						InterThread::setGuiWorkLoop(muscleForce, data, lmtMaTime);
						InterThread::setGuiTimeTorque(lmtMaTime);
						InterThread::AddToNbFrameNMS(1);
						timeNow = rtb::getTime();
						InterThread::AddToTimeConsumeNMS(timeNow - timeConsume);
					}

#endif

					lmtMaTimePast = lmtMaTime;
				}

#ifdef VERBOSE
				else
				{
					if (_verbose > 2)
						COUT << "Empty Lmt ot Ma vector" << std::endl;
				}

#endif
			}
		}

#ifdef VERBOSE

		if (_verbose > 2)
			COUT << "NMS: done ..." << std::endl;

#endif
		// notify the producer
		InterThread::NMSconsumerDoneEMG.notify();


#ifdef TIMING
		timeDoubleEnd = rtb::getTime();
		timeDoubleCurrent = timeDoubleEnd - timeDoubleBegin;
		timeDoubleTotal += timeDoubleCurrent;
#endif
	}

#ifdef TIMING
	//     SyncTools::Shared::lmtTimeMutex.lock();
	COUT << "TimingNMS Mean: " << timeDoubleTotal / cptTotal * 1000 << " ms." << std::endl << std::flush;
	COUT << "TimingNMSWait Mean: " << timeDoubleTotalWait / cptTotal * 1000 << " ms." << std::endl << std::flush;
	//     SyncTools::Shared::lmtTimeMutex.unlock();
#endif
	//std::cout << "NMS sleep reached \n";
	//std::this_thread::sleep_for(std::chrono::hours(4));
#ifndef NO_SAVE
	if (pluginOptimizationBool)
	{

		_optimizationPlugin.getPlugin()->stop();
		_optimizationPlugin.closeDynLib();
	}

	if (_record) // stop save to file
	{
		logger->stop();
		delete logger;
	}

#endif

	if (pluginBool) // stop plugin
	{
#ifdef VERBOSE

		if (_verbose > 1)
			COUT << "stop Comsumer plugin" << std::endl;

#endif
		plugin_->stop();
		closeDynLib();
}

	if (!_useOfEmgAndAngleAndComsumerPlugin)
	{
		_emgPlugin.getPlugin()->stop();
		_emgPlugin.closeDynLib();
	}


#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "\033[1;32mNMS thread end\033[0m" << std::endl;

#endif
	}

template<typename NMSmodelT>
void ModelEvaluationRealTime<NMSmodelT>::WaitForProducer()
{
#ifdef VERBOSE

	if (_verbose > 2)
		COUT << "NMS: waiting for LMT ..." << std::endl;

#endif

	InterThread::lmtProducingDone.wait();

#ifdef VERBOSE

	if (_verbose > 2)
		COUT << "NMS: getting data LMT ..." << std::endl;

#endif
}

template<typename NMSmodelT>
void ModelEvaluationRealTime<NMSmodelT>::getEMG(std::vector<double>& EMGData, double& emgTime)
{
	EMGData.clear();
	/*const map<string, double>& mapData;
	if (!_useOfEmgAndAngleAndComsumerPlugin)
		mapData = _emgPlugin.getPlugin()->GetDataMap(); // get data from EMG plugin
	else
		mapData = _pluginEAC->GetDataMapEMG(); // get data from EMG, angle and comsumer plugin*/
		// If statement is not possible because of the reference. The following line does the same:
		// If we use EmgAndAngleAndComsumerPlugin, we get the mapData from the EACplugin, otherwise we get the mapData from EMG plugin
		//std::cout << "getEMG() called" << std::endl;
	const map<string, double>& mapData = (!_useOfEmgAndAngleAndComsumerPlugin) ? _emgPlugin.getPlugin()->GetDataMap() : _pluginEAC->GetDataMapEMG();

	map<string, double> mapMuscle;

	// We have channel EMG we need muscle EMG, so we distribute EMG
	for (UnMapStrVectStr::const_iterator it1 = channelNameOnMuscle_.begin(); it1 != channelNameOnMuscle_.end();
		it1++)
	{
		double data = 0;

		if (it1->second.size() != 0)
		{
			//std::cout << it1->first << ": ";

			for (vector<string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++) // Some muscle are on two or more channel
			{


				try
				{
					//std::cout << *it2 << "\t";
					data += mapData.at(*it2);

                }
                catch (const std::out_of_range& oor)
                {
                    COUT << "Channel " << *it2 << " not found in the EMG file." << std::endl;

					InterThread::setEndThread(true);
					break;
				}

			}

			data /= it1->second.size(); // mean of all the channel use by the muscle

		}

		mapMuscle[it1->first] = data;

		//std::cout << it1->first << " : " << data << std::endl;
	}

	// Sort the EMG data to have the same order as the muscles from the model
	for (vector<string>::const_iterator it1 = muscleNames_.begin(); it1 != muscleNames_.end(); it1++)
	{
		if (mapMuscle.find(*it1) != mapMuscle.end())
		{
			EMGData.push_back(mapMuscle.at(*it1));
			//std::cout << *it1 << ": " << mapMuscle.at ( *it1 ) << std::endl;
		}
		else
			EMGData.push_back(0);
	}

	if (!_useOfEmgAndAngleAndComsumerPlugin)
		emgTime = _emgPlugin.getPlugin()->getTime();
	else
		emgTime = _pluginEAC->getTime();

	InterThread::setEMGTime(emgTime);
	//std::cout << emgTime << std::endl;
}

template<typename NMSmodelT>
void ModelEvaluationRealTime<NMSmodelT>::initEMGPlugin()
{
	std::string dynLib;
	ExecutionXmlReader xml(_executionName);

	if (!_process) // if we do not read EMG from file
	{
		dynLib = xml.getEmgPlugin();
	}
	else
	{
#ifdef WINDOWS
		dynLib = "PluginEMGFiltFromFile.dll"; // Overrule the xml, name of the read from file plugin
#else
#ifdef LINUX
		dynLib = "PluginEMGFiltFromFile.so"; // Overrule the xml, name of the read from file plugin
#endif
#endif
	}

	if (!_useOfEmgAndAngleAndComsumerPlugin)
	{
		if (!_emgPlugin.setDynLib(dynLib)) {
			std::cerr << "Failed to load library: " << dynLib << std::endl;
			exit(1);
		}

		if (_record) // give the directory for the recoding on file and also the directory where the file we read from are if needed
		{
			if (_process)
				_emgPlugin.getPlugin()->setDirectories(_recordDirectory, _processDirectory);
			else
				_emgPlugin.getPlugin()->setDirectories(_recordDirectory);
		}
		else if (_process)
		{
			_emgPlugin.getPlugin()->setDirectories("output", _processDirectory);
		}


		_emgPlugin.getPlugin()->setVerbose(_verbose); // set the level of verbose for the plugin
		_emgPlugin.getPlugin()->setRecord(_record); // set if we record data on file

		boost::filesystem::path full_path(boost::filesystem::current_path());
		_emgPlugin.getPlugin()->init(xmlName_, _executionName);
		boost::filesystem::current_path(full_path);
	}
	MapStrVectStr musclesNamesOnChannel = InterThread::getMusclesNamesOnChannel(); // get the repartition of the muscles on the differrent DOF from the model
	MapStrVectStr channelNameOnMuscleTemp;

	// Create a map between muscle and channel.
	for (MapStrVectStr::const_iterator it1 = musclesNamesOnChannel.begin(); it1 != musclesNamesOnChannel.end(); it1++)
		for (std::vector<std::string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++)
			channelNameOnMuscleTemp[*it2].push_back(it1->first);

	getMusclesNames(muscleNames_); // get the muscle name from the model, order is also important

	if (muscleNames_.size() == 0) // if the mucle name vector is empty we fill it we data fom the muscle on DOF
	{
		for (MapStrVectStr::const_iterator it1 = channelNameOnMuscleTemp.begin(); it1 != channelNameOnMuscleTemp.end();
			it1++)
			muscleNames_.push_back(it1->first);
	}

	setMusclesNames(muscleNames_); // we set it for the other thread

	// Use musclesNames and unordered boost map for having the same correspondence between the emg and the muscle.
	for (std::vector<std::string>::const_iterator it1 = muscleNames_.begin(); it1 != muscleNames_.end(); it1++)
	{
		try
		{
			channelNameOnMuscle_[*it1] = channelNameOnMuscleTemp.at(*it1);
		}
		catch (const std::out_of_range& oor)
		{
#ifdef VERBOSE

			if (_verbose > 1)
				cout << *it1 << " will be put to zero because it was not found in the channel" << std::endl;

#endif
		}
	}
}
