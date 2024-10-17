/*
 * LmtMaFromMTUSpline.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: gdurandau
 */

#include "LmtMaFromMTUSpline.h"

LmtMaFromMTUSpline::LmtMaFromMTUSpline(const std::string& subjectSpecificXml,
	const std::string& subjectName, string executionName, bool record, std::string recordDirectory,
	std::string processDirectory, bool process) :
	subjectSpecificXml_(subjectSpecificXml), subjectName_(subjectName), _record(record),
	_recordDirectory(recordDirectory), executionName_(executionName), _processDirectory(processDirectory),
	_process(process), _useOfAngleAndComsumerPlugin(false), _useOfEmgAndAngleAndComsumerPlugin(false), _lmtTimePast(-1), _verbose(0)
{

}

LmtMaFromMTUSpline::~LmtMaFromMTUSpline()
{
#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "\033[1;31mMTU " << this << "\033[0m" << std::endl;

#endif
}

void LmtMaFromMTUSpline::operator() ()
{

	std::vector<double> angleFromQueue;
	std::vector<double> lmtForQueue;
	std::vector<double> idData;
	std::vector<std::string> dofNames;
	std::vector<std::string> muscleNames;
	std::vector<std::vector<double> > maForQueue;

#ifdef UNIX
	timeval now;
	timeval timeConsumeStruct;
#endif

	double timeNow;
	double timeConsume;
	double lmtTimePast = -1;
	double angleTime = -1;

	bool firstPass = true;


#ifdef TIMING
	timeval timeBeginWait, timeBegin, timeEnd;
	double timeDoubleBegin = 0, timeDoubleEnd = 0, timeDoubleTotal = 0, timeDoubleCurrent = 0;
	double timeDoubleBeginWait = 0, timeDoubleTotalWait = 0, timeDoubleCurrentWait = 0;
	int cptTotal = 0;
#endif

	initAnglePlugin();

	getSyncVerbose(_verbose);
	getSyncGui(_gui);

#ifndef NO_SAVE
	OpenSimFileLogger<int>* logger;

	if (_record)
	{
		logger = new OpenSimFileLogger<int>(_recordDirectory);
		logger->addLog(Logger::MTUTiming);
	}

#endif

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "LMT: waiting for dof names..." << endl;

#endif

	getDofNames(dofNames);


#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "LMT: waiting for muscle names..." << endl;

#endif

	getMuscleNamesFromShared(muscleNames);

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "LMT: waiting for muscle names... Done" << endl;

#endif

	lmtForQueue.resize(muscleNames.size());

	MTUSplineDataRead* splineData = new MTUSplineDataRead(subjectSpecificXml_, subjectName_);

	splineData->readTaskCoefficients();

	std::vector<MTUSplineDataRead::Task> taskVect = splineData->getTaskVect();

	for (vector<string>::const_iterator it1 = dofNames.begin(); it1 != dofNames.end(); it1++)
		musclesNamesOnDof_.push_back(splineData->getMuscleOnDof(*it1));

	delete splineData;

	setMusclesNamesOnDof(musclesNamesOnDof_);

	maForQueue.resize(dofNames.size());

	for (vector<vector<string> >::iterator it1 = musclesNamesOnDof_.begin(); it1 != musclesNamesOnDof_.end(); it1++)
		maForQueue[distance<vector<vector<string> >::iterator>(musclesNamesOnDof_.begin(), it1)].resize(it1->size());

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "LMT: waiting for ready to start..." << endl;

#endif

	InterThread::readyToStart->wait(); // Wait for ModelEvaluationRealTime and GUI

	while (true)
	{
		if (InterThread::getEndThread()) // stop condition
		{
			InterThread::notifyAll();
			// 			COUT << "stop" << std::endl << std::flush;
			break;
		}


#ifdef VERBOSE

		if (_verbose > 2)
			COUT << "LMT: waiting for producers..." << endl;

#endif

#ifdef TIMING
		timeDoubleBeginWait = rtb::getTime();
#endif

		// 		InterThread::angleProducingDone.wait();

		if (firstPass) // First get the angle Data
		{
			//	COUT << "0" << std::endl;
			firstPass = false;
			getAngle(angleFromQueue, idData, angleTime);

			//	COUT << "1" << std::endl;
	// 			SyncTools::Shared::lmtTimeMutex.lock();
	// 			InterThread::setLmtTime(-1);
	// 			SyncTools::Shared::lmtTimeMutex.unlock();
		}
		else if (angleTime >= 0) // if angle time is =< 0, it's means that we have no angle data
		{
			if (_verbose > 2)
				COUT << "LMT: Waiting for ModelEvaluationRealTime to finish..." << endl;
			InterThread::NMSconsumerDoneAngle.wait(); // wait for ModelEvaluationRealTime to finish
		}

#ifdef TIMING
		timeDoubleBegin = rtb::getTime();
		timeDoubleCurrentWait = timeDoubleBegin - timeDoubleBeginWait;
		timeDoubleTotalWait += timeDoubleCurrentWait;
		cptTotal++;
#endif

#ifdef VERBOSE

		/*	if ( _verbose > 2 )
				COUT << "LMT: waiting for angle..." << std::endl;*/

#endif

				// Get the last capture time of the EMg and Lmt

		double emgTime;
		double lmtTime = InterThread::getLmtTime();

		// Update lmt time with last angle capture time
		InterThread::setLmtTime(angleTime);

		do
			emgTime = InterThread::getEMGTime();
		while (!(angleTime <= emgTime || !_process) && !InterThread::getEndThread());

		//if (angleTime <= emgTime || !_process) // if last angle capture time is inferior to the last capture time of the EMG we capture some angle data
												// Process is the bool for processing from file
		//{

		do
			getAngle(angleFromQueue, idData, angleTime); //update angle capture data
		while (!(lmtTimePast != angleTime && angleTime >= 0) && !InterThread::getEndThread());


		//if (lmtTimePast != angleTime && angleTime >= 0) // if the angle data is updated and superior to zero (valide data)
		//{

			// get the current time
		timeConsume = rtb::getTime();

		for (std::vector<MTUSplineDataRead::Task>::iterator it1 =
			taskVect.begin(); it1 != taskVect.end(); it1++)
		{

			std::vector<double> lmt;
			std::vector<std::vector<double> > ma;
			std::vector<double> angle;


			// sort angle data to correspond to the BSpline DOF list
			for (set<string>::const_iterator it2 = it1->uniqueDOFlist.begin(); it2 != it1->uniqueDOFlist.end(); it2++)
				for (vector<string>::const_iterator it3 = dofNames.begin(); it3 != dofNames.end(); it3++)
				{
					if (*it2 == *it3)
					{
						angle.push_back(angleFromQueue[distance < vector<string>::const_iterator >(dofNames.begin(), it3)]);
						//std::cout << *it3 << " : " << angle.back() <<std::endl << std::flush;
						break;
					}
				}

			noMuscles_ = it1->uniqueMuscleList.size();

			switch ((*it1).uniqueDOFlist.size()) // Do BSpline magic depending of the order of the spline
			{
			case 1:
			{
				vector<std::shared_ptr<MTUSpline<1> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<1>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, angle, lmt, ma);
				break;
			}

			case 2:
			{
				vector<std::shared_ptr<MTUSpline<2> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<2>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 2, angle, lmt, ma);
				break;
			}

			case 3:
			{
				vector<std::shared_ptr<MTUSpline<3> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<3>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 3, angle, lmt, ma);
				break;
			}

			case 4:
			{
				vector<std::shared_ptr<MTUSpline<4> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<4>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 4, angle, lmt, ma);
				break;
			}

			case 5:
			{
				vector<std::shared_ptr<MTUSpline<5> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<5>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 5, angle, lmt, ma);
				break;
			}

			case 6:
			{
				vector<std::shared_ptr<MTUSpline<6> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<6>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 6, angle, lmt, ma);
				break;
			}

			case 7:
			{
				vector<std::shared_ptr<MTUSpline<7> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<7>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 7, angle, lmt, ma);
				break;
			}

			case 8:
			{
				vector<std::shared_ptr<MTUSpline<8> > > temp;
				temp.resize((*it1).splines_.size());

				for (int i = 0; i < (*it1).splines_.size(); i++)
					temp[i] = std::dynamic_pointer_cast<MTUSpline<8>> ((*it1).splines_[i]);

				computeLmtMafromSplines(temp, 8, angle, lmt, ma);
				break;
			}

			default:
#ifdef VERBOSE
				if (_verbose > 0)
					COUT << "The dimension is bigger than 8 which is the maximum accepted." << endl;

#endif
				exit(1);
			}

			// Sort Lmt data to follow the model muscle sorting
			for (vector<string>::const_iterator it2 = muscleNames.begin(); it2 != muscleNames.end(); it2++)
			{
				//vector<string>::iterator it3 = it1->uniqueMuscleList.find(*it2);
				vector<string>::iterator it3 = find(it1->uniqueMuscleList.begin(), it1->uniqueMuscleList.end(), *it2);

				if (it3 != it1->uniqueMuscleList.end())
				{
					lmtForQueue[distance<vector<string>::const_iterator>(muscleNames.begin(), it2)] = lmt[distance <
						vector<string>::iterator >(it1->uniqueMuscleList.begin(), it3)];
					//std::cout << *it2 << lmt[distance <set<string>::iterator >(it1->uniqueMuscleList.begin(), it3)] << std::endl << std::flush;
				}
			}

			// sort MA data to follow the model muscle on DOF sorting
			for (std::vector<std::vector<std::string> >::const_iterator it2 = musclesNamesOnDof_.begin();
				it2 != musclesNamesOnDof_.end(); it2++)
			{
				set<string>::iterator it4 =
					it1->uniqueDOFlist.find(dofNames[distance < std::vector<std::vector<std::string> >::const_iterator >(
						musclesNamesOnDof_.begin(), it2)]);

				if (it4 != it1->uniqueDOFlist.end())
				{
					for (vector<string>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++)
					{

						//vector<string>::iterator it5 = it1->uniqueMuscleList.find(*it3);
						vector<string>::iterator it5 = find(it1->uniqueMuscleList.begin(), it1->uniqueMuscleList.end(), *it3);

						if (it5 != it1->uniqueMuscleList.end())
						{
							maForQueue[distance < std::vector<std::vector<std::string> >::const_iterator >(musclesNamesOnDof_.begin(), it2)][distance <
								vector<string>::const_iterator >(it2->begin(), it3)] = ma[distance<set<string>::iterator>(
									it1->uniqueDOFlist.begin(), it4)][distance < vector<string>::iterator >(it1->uniqueMuscleList.begin(), it5)];
						}
					}
				}
			}
		}

		// Update MA
		for (std::vector<std::vector<std::string> >::const_iterator it2 =
			musclesNamesOnDof_.begin(); it2 != musclesNamesOnDof_.end(); it2++)
		{
			updateMomentArms(maForQueue[distance <
				std::vector<std::vector<std::string> >::const_iterator >(musclesNamesOnDof_.begin(), it2)], angleTime,
				distance < std::vector<std::vector<std::string> >::const_iterator >(musclesNamesOnDof_.begin(), it2));
		}

		// Update Lmt
		updateLmt(lmtForQueue, angleTime);
		InterThread::setGuiTimeLMT(angleTime);
		InterThread::setLmtTime(angleTime); // update Lmt time

#ifndef NO_GUI
		timeNow = rtb::getTime();

		if (_gui) // Update GUI number of angle frames processed and computational time
		{
			InterThread::AddToNbFrameMTU(1);
			InterThread::AddToTimeConsumeMTU(timeNow - timeConsume);
		}

#endif
#ifndef NO_SAVE

		if (_record) // record data on file
		{
			double data = timeNow - angleTime;
			logger->log(Logger::MTUTiming, angleTime, data);
		}

#endif
		lmtTimePast = angleTime;
		//}
	//}
/*
else // if we don't have new data just use the old one
	{
		for (std::vector<std::vector<std::string> >::const_iterator it2 =
			musclesNamesOnDof_.begin(); it2 != musclesNamesOnDof_.end(); it2++)
		{
			updateMomentArms(maForQueue[distance <
				std::vector<std::vector<std::string> >::const_iterator >(
					musclesNamesOnDof_.begin(), it2)], lmtTimePast,
				distance < std::vector<std::vector<std::string> >::const_iterator >(
					musclesNamesOnDof_.begin(), it2));
		}

		updateLmt(lmtForQueue, lmtTimePast);

	} //*/

#ifdef VERBOSE

		if (_verbose > 2)
			COUT << "LMT: Producing Done..." << endl;

#endif
		// Notify that we have new lmt and Ma data
		InterThread::lmtProducingDone.notify();

#ifdef TIMING
		timeDoubleEnd = rtb::getTime();
		timeDoubleCurrent = timeDoubleEnd - timeDoubleBegin;
		timeDoubleTotal += timeDoubleCurrent;
#endif
	}

#ifdef TIMING
	COUT << "TimingMTU Mean: " << timeDoubleTotal / cptTotal * 1000 << " ms." << std::endl << std::flush;
	COUT << "TimingMTUWait Mean: " << timeDoubleTotalWait / cptTotal * 1000 << " ms." << std::endl << std::flush;
#endif

#ifndef NO_SAVE

	if (_record) // Stop the recording on file
	{
		logger->stop();
		delete logger;
	}
	//std::cout << "MTU sleep reached \n";
	//std::this_thread::sleep_for(std::chrono::hours(4));
	if (_pluginBool) // Stop the plugin
	{
		_anglePlugin.getPlugin()->stop();
		std::cout << "anglePlugin stopped" << std::endl;
		//_anglePlugin.closeDynLib(); // crash when using -p option
	}

#endif

#ifdef VERBOSE

	if (_verbose > 1)
		COUT << "\033[1;32mMTU thread end\033[0m" << std::endl;

#endif
}

template<class T>
void LmtMaFromMTUSpline::computeLmtMafromSplines(T& splines, int dim,
	const std::vector<double>& angles, std::vector<double>& lmt,
	std::vector<std::vector<double> >& ma)
{
	lmt.clear();
	ma.clear();

	for (int i = 0; i < noMuscles_; ++i)
		lmt.push_back(splines[i]->getValue(angles));

	ma.resize(dim);

	for (int k = 0; k < dim; ++k)
		for (int i = 0; i < noMuscles_; ++i)
			ma[k].push_back(-splines[i]->getFirstDerivative(angles, k));
}

void LmtMaFromMTUSpline::computeLmtMafromSplines(
	std::vector<std::shared_ptr<MTUSpline<1> > >& splines,
	const std::vector<double>& angles, std::vector<double>& lmt,
	std::vector<std::vector<double> >& ma)
{
	lmt.clear();
	ma.clear();

	for (int i = 0; i < noMuscles_; ++i)
		lmt.push_back(splines[i]->getValue(angles[0]));

	ma.resize(1);

	for (int i = 0; i < noMuscles_; ++i)
		ma[0].push_back(-splines[i]->getFirstDerivative(angles[0]));
}

void LmtMaFromMTUSpline::getAngle(std::vector<double>& angleData, std::vector<double>& idData, double& angleTime)
{
	angleData.clear();
	idData.clear();
	std::map<std::string, double> mapData;
	std::map<std::string, double> mapIdData;

	if (_pluginBool) // if we use an external pludin
	{
		mapData = _anglePlugin.getPlugin()->GetDataMap(); // Data from plugin is a map in case the dof are in different order 
		mapIdData = _anglePlugin.getPlugin()->GetDataMapTorque(); // get the DOF torque compute using sensor or inverse dynamic
		angleTime = _anglePlugin.getPlugin()->getTime(); // time of the recording of the angle

		if (angleTime == _lmtTimePast && _process) 	// if we read from file (bool process) and we have the same time of reccording it means that the file is finish
			InterThread::setEndThread(true);

		_lmtTimePast = angleTime;

#ifdef VERBOSE

		if (_verbose > 2)
			COUT << "Angle time: " << _anglePlugin.getPlugin()->getTime() << std::endl;

#endif

	}
	else if (_useOfAngleAndComsumerPlugin) // use of external plugin that also get the EMG-driven Torque given by ModelEvaluationRealTime
	{
		angleTime = _pluginAAC->GetAngleTimeStamp(); // time of the recording of the angle
		mapData = _pluginAAC->GetDataMap(); // Data from plugin is a map in case the dof are in different order 
		mapIdData = _pluginAAC->GetDataMapTorque(); // get the DOF torque compute using sensor or inverse dynamic
	}
	else if (_useOfEmgAndAngleAndComsumerPlugin)
	{
		//angleTime = _pluginEAC->GetAngleTimeStamp(); // time of the recording of the angle
		angleTime = _pluginEAC->getTime();
		mapData = _pluginEAC->GetDataMapAngle(); // Data from plugin is a map in case the dof are in different order 
		mapIdData = _pluginEAC->GetDataMapTorque(); // get the DOF torque compute using sensor or inverse dynamic
	}

	// sort the data from map to vector following the model DOF order
	for (vector<string>::const_iterator it1 = dofNames_.begin(); it1 != dofNames_.end(); it1++)
	{
		if (mapData.find(*it1) != mapData.end())
			angleData.push_back(mapData.at(*it1));
		else
		{
			//COUT << "Joint " << *it1 << " not found in the IK result." << std::endl << std::flush;
			//InterThread::setEndThread ( true );
			angleData.push_back(0);
		}

		if (mapIdData.find(*it1) != mapIdData.end())
		{
			idData.push_back(mapIdData.at(*it1));
			//std::cout << "Joint " << *it1 << " : " << idData.back() << std::endl << std::flush;
		}
		else
		{
			idData.push_back(0);
			//std::cout << "Joint " << *it1 << " not found in the ID result." << std::endl << std::flush;
		}
	}

	InterThread::setExternalTorque(idData);
	InterThread::setAngle(angleData);

	if (_process && _gui) // if we process from file and we use a Gui we slow down so we can visualise
		std::this_thread::sleep_for(std::chrono::microseconds(7500));

	if (_gui) // send data to the GUI
	{
		InterThread::setPositionMap(mapData);
		InterThread::setGuiID(idData);
	}

}

void LmtMaFromMTUSpline::initAnglePlugin()
{
	std::vector<std::string> dofNamePlugin;
	std::string dynLib;

	ExecutionXmlReader xml(executionName_);

	getDofNames(dofNames_); // get dof name from the model
	getMuscleNamesFromShared(muscleNames_);

	if (!_process) //if we are not processing from file
	{
		_useOfAngleAndComsumerPlugin = xml.useOfAngleAndComsumerPlugin();
		_pluginBool = xml.useOfAnglePlugin();
		if (xml.useOfEmgAndAngleAndComsumerPlugin())
		{
			_useOfEmgAndAngleAndComsumerPlugin = true;
			_useOfAngleAndComsumerPlugin = false;
		}
	}
	else // we process from extrenal plugin
		_pluginBool = true;


	if (_useOfAngleAndComsumerPlugin)
	{
		_pluginAAC->setDofName(dofNames_);
        _pluginAAC->setMuscleName(muscleNames_);
		_pluginAAC->init(executionName_);

		dynLib = xml.getAngleAndComsumerPlugin();
		_pluginBool = false;

		try
		{
			std::auto_ptr<ExecutionIKType> executionPointerIK(executionIK(xml.getAngleFile(), xml_schema::flags::dont_initialize));
			InterThread::setModelFileName(executionPointerIK->OsimFile()); // get the OpenSIm model name for the GUI
		}
		catch (const xml_schema::exception& e)
		{
			COUT << e << endl;
			exit(EXIT_FAILURE);
		}
	}
	else if (_useOfEmgAndAngleAndComsumerPlugin)
	{
		_pluginEAC->setDofName(dofNames_);
		_pluginEAC->init(subjectSpecificXml_, executionName_);

		dynLib = xml.getEmgAndAngleAndComsumerPlugin();
		_pluginBool = false;

		try
		{
			std::auto_ptr<ExecutionIKType> executionPointerIK(executionIK(xml.getAngleFile(), xml_schema::flags::dont_initialize));
			InterThread::setModelFileName(executionPointerIK->OsimFile()); // get the OpenSIm model name for the GUI
		}
		catch (const xml_schema::exception& e)
		{
			COUT << e << endl;
			exit(EXIT_FAILURE);
		}
	}

	if (_pluginBool)
	{
		//std::stringstream ssCheckPresence; // added by Thijs to double check
		//ssCheckPresence << "./" << _processDirectory << "/id.sto";
		//std::ifstream dataFileCheckPresence{ ssCheckPresence.str().c_str() };

		if (!_process) //|| dataFileCheckPresence.is_open() )
		{
			//dataFileCheckPresence.close();
			dynLib = xml.getAnglePlugin();
		}
		else
		{
#ifdef WINDOWS
			dynLib = "PluginAngleAndIDFromFile.dll"; // Overrule the xml, name of the read from file plugin
#else
#ifdef LINUX
			dynLib = "PluginAngleAndIDFromFile.so"; // Overrule the xml, name of the read from file plugin
#endif
#endif
		}

		try
		{
			std::auto_ptr<ExecutionIKType> executionPointerIK(executionIK(xml.getAngleFile(), xml_schema::flags::dont_initialize));
			InterThread::setModelFileName(executionPointerIK->OsimFile()); // get the OpenSIm model name for the GUI
		}
		catch (const xml_schema::exception& e)
		{
			COUT << e << endl;
			exit(EXIT_FAILURE);
		}
	}

	if (_pluginBool)
	{
		std::cout << dynLib << std::endl;
		_anglePlugin.setDynLib(dynLib);

		if (_record)
		{
			if (_process)
				_anglePlugin.getPlugin()->setDirectories(_recordDirectory, _processDirectory); // set directorie for the recording on file and the file that we will read from
			else
				_anglePlugin.getPlugin()->setDirectories(_recordDirectory); // set directorie for the recording on file

		}
		else if (_process)
			_anglePlugin.getPlugin()->setDirectories("output", _processDirectory); // set directorie for the recording on file and the file that we will read from

		_anglePlugin.getPlugin()->setVerbose(_verbose); // if we want a talkative plugin
		_anglePlugin.getPlugin()->setRecord(_record); // if we record on file

		boost::filesystem::path full_path(boost::filesystem::current_path());

		_anglePlugin.getPlugin()->init(subjectSpecificXml_, executionName_); // init the plugin with execution XML and CEINMS subject XML

		boost::filesystem::current_path(full_path);
	}
	/*else if ( _useOfAngleAndComsumerPlugin )
	{
		COUT << "setDOF" << std::endl;
		_pluginAAC->setDofName ( dofNames_ ); // set the name of the DOF from the model
		dofNamePlugin = _pluginAAC->GetDofName();
	}*/
}
