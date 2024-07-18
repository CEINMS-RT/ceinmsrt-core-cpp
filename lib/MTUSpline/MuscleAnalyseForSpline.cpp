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

#include "MuscleAnalyseForSpline.h"
//#define PRINT_DOF_LIST

#define PRECISION 6

#ifdef USE_OPENSIM

MuscleAnalyseForSpline::MuscleAnalyseForSpline( const string& configurationFile, const string& OsimModelName, const int& nbOfStep,
		const int& printOption) :
		configurationFile_(configurationFile), osimModel_(Model(OsimModelName.c_str())), si_(osimModel_.initSystem()), nbOfStep_(nbOfStep)
{
	if(printOption == 1)
	{
		//cout << "print only CEINMS file" << endl;
		printCEINMSData_ = true;
		printOsimData_ = false;
	}
	else if(printOption == 2)
	{
		//cout << "print only OpenSim file" << endl;
		printCEINMSData_ = false;
		printOsimData_ = true;
	}
	else if(printOption == 3)
	{
		//cout << "print OpenSim and CEINMS file" << endl;
		printCEINMSData_ = true;
		printOsimData_ = true;
	}
	else
	{
		//cout << "print only coefficients" << endl;
		printCEINMSData_ = false;
		printOsimData_ = false;
	}
	init(configurationFile);
}

MuscleAnalyseForSpline::MuscleAnalyseForSpline( const string& configurationFile, const string& OsimModelName, const string& translateFileName, const int& nbOfStep,
		const int& printOption) :
		configurationFile_(configurationFile), nbOfStep_(nbOfStep)
{

	std::cout << OsimModelName << std::endl << std::flush;
	osimModel_ = Model(OsimModelName);
	si_ = SimTK::State(osimModel_.initSystem());

	if(printOption == 1)
	{
		//cout << "print only CEINMS file" << endl;
		printCEINMSData_ = true;
		printOsimData_ = false;
	}
	else if(printOption == 2)
	{
		//cout << "print only OpenSim file" << endl;
		printCEINMSData_ = false;
		printOsimData_ = true;
	}
	else if(printOption == 3)
	{
//		cout << "print OpenSim and CEINMS file" << endl;
		printCEINMSData_ = true;
		printOsimData_ = true;
	}
	else
	{
		//cout << "print only coefficients" << endl;
		printCEINMSData_ = false;
		printOsimData_ = false;
	}

	init(configurationFile);
}

void MuscleAnalyseForSpline::init(const string& configurationFile)
{
	try
	{
		std::auto_ptr<NMSmodelType> subjectPointer(subject(configurationFile));
		NMSmodelType::muscles_type& muscles(subjectPointer->muscles());
		MusclesType::muscle_sequence& muscleSequence(muscles.muscle());

		for (MusclesType::muscle_iterator i(muscleSequence.begin()); i != muscleSequence.end(); ++i)
		{
			muscleNameListOpenSim_.append((*i).name());
			vectMuscleNameListOpenSim_.push_back((*i).name());
			vectMuscleNameListCEINMS_.push_back((*i).name());
		}

		// DOF iteration
		NMSmodelType::DoFs_type& dofs(subjectPointer->DoFs());
		DoFsType::DoF_sequence& dofSequence(dofs.DoF());
		for (DoFsType::DoF_iterator i = dofSequence.begin(); i != dofSequence.end(); ++i)
		{
			string currentDOF((*i).name());
			DOFNameListOpenSim_.append(currentDOF);
			vectDOFNameListOpenSim_.push_back(currentDOF);
			vectDOFNameListCEINMS_.push_back(currentDOF);

			// iteration in the muscle containing in each DOF. See end of subject specific XML.
			MuscleSequenceType currentSequence = (*i).muscleSequence();
			MuscleSequenceType::iterator muscleIt;
			for (muscleIt = currentSequence.begin(); muscleIt != currentSequence.end(); ++muscleIt)
			{
				string currentMuscle(*muscleIt);
				DOFMuscleConnect_.insert(position(currentDOF, currentMuscle));
			}
		}
	} catch (const xml_schema::exception& e)
	{
		cout << e << endl;
		exit(EXIT_FAILURE);
	}

	modelName_ = osimModel_.getName();

	//cout << "end init" << endl;
//	printMemUsage();

}

MuscleAnalyseForSpline::~MuscleAnalyseForSpline()
{
}

void MuscleAnalyseForSpline::computeAnglesStorage()
{
	//cout << "compute angles" << endl;
	// Create unique list of unique DOF name list that is cross by the same muscle.
	multimap<set<string>, string> DOFsetToMuscle;
	set<set<string> > uniqueDOFlist;
	int maxSizeSet;
	for (vector<string>::const_iterator it = vectMuscleNameListOpenSim_.begin(); it != vectMuscleNameListOpenSim_.end();
			it++)
	{
		string muscleName(*it);
		set<string> temp;
		for (DOFMuscleMap::right_const_iterator itBimap = DOFMuscleConnect_.right.lower_bound(muscleName);
				itBimap != DOFMuscleConnect_.right.upper_bound(muscleName); itBimap++)
			temp.insert(itBimap->second);
		DOFsetToMuscle.insert(pair<set<string>, string>(temp, *it));
		uniqueDOFlist.insert(temp);
	}

	taskVect_.resize(uniqueDOFlist.size());
	for (set<set<string> >::const_iterator it1 = uniqueDOFlist.begin(); it1 != uniqueDOFlist.end(); it1++)
	{
		taskVect_[distance<set<set<string> >::const_iterator>(uniqueDOFlist.begin(), it1)].uniqueDOFlist.insert((*it1).begin(), (*it1).end());
		for(multimap<set<string>, string>::const_iterator it2 = DOFsetToMuscle.lower_bound(*it1); it2 != DOFsetToMuscle.upper_bound(*it1); it2++)
			taskVect_[distance<set<set<string> >::const_iterator>(uniqueDOFlist.begin(), it1)].uniqueMuscleList.push_back(it2->second);
	}

	// print the groups of DOF.

	vector<Task> takVectTemp;
	for (vector<Task>::iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++)
		if((*it1).uniqueDOFlist.size() != 0)
			takVectTemp.push_back(*it1);
	taskVect_ = takVectTemp;

#ifdef PRINT_DOF_LIST
	cout << "Group of DOF found:" << endl;
	for (vector<Task>::iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++)
	{
		for (set<string>::iterator it2 = (*it1).uniqueDOFlist.begin(); it2 != (*it1).uniqueDOFlist.end(); it2++)
			cout << *it2 << " ";
		cout << "\t:\t";
		for (set<string>::iterator it2 = (*it1).uniqueMuscleList.begin(); it2 != (*it1).uniqueMuscleList.end(); it2++)
			cout << *it2 << " ";
		cout << endl;
	}
	cout << endl;
#endif




//	cout << "Get the min, max and increments of DOF" << endl;
	// Get the min, max and increments of DOF.
	double range;
	for (vector<string>::const_iterator it = vectDOFNameListOpenSim_.begin(); it != vectDOFNameListOpenSim_.end(); it++)
	{
		minDOF_[*it] = getMinAnglesDof(*it);
		maxDOF_[*it] = getMaxAnglesDof(*it);
		range = maxDOF_.at(*it) + abs(minDOF_.at(*it));
		increments_[*it] = range / (nbOfStep_);
	}

//	cout << "fill with the min range DOF" << endl;
	// fill with the min range DOF that not in the subset of outputDOFlist
	//(but still in the subject specific XML) and put then at the back.
	for (vector<Task>::iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++)
	{
//		cout << distance<vector<Task>::iterator>(taskVect_.begin(), it1) << endl;
		vector<vector<double> > anglesTemp;
		vector<vector<double> > anglesBetweenNode;
		anglesTemp.resize((*it1).uniqueDOFlist.size());
		anglesBetweenNode.resize((*it1).uniqueDOFlist.size());
		for (set<string>::const_iterator it2 = (*it1).uniqueDOFlist.begin(); it2 != (*it1).uniqueDOFlist.end();
				it2++)
		{
			/*
			 * The angles creation creates probleme at the limit for avoiding
			 *  it the best solution is to create smooth transition like this:
			 *  now: -1 -0.5 0 0.5 1 -1-0.5 0 0.5 1
			 *  better: -1 -0.5 0 0.5 1 0.5 0 -0.5 -1 and so on
			 *  will double the size of the computation
			 */
			int iteration = distance<set<string>::const_iterator>((*it1).uniqueDOFlist.begin(), it2);
			int maxIteration = (*it1).uniqueDOFlist.size();
//			cout << "pow: " << pow(nbOfStep_ + 1, (maxIteration - iteration - 1)) << endl;
//			cout << "pow2: " << pow(nbOfStep_, (maxIteration - iteration - 1)) << endl;
			for (int i = 0; i < pow(nbOfStep_ + 1, (maxIteration - iteration - 1)); i++) // column length
				for (int j = 0; j < nbOfStep_ + 1; j++)
					for (int k = 0; k < pow(nbOfStep_ + 1, iteration); k++)
					{
//						if(i % 2 == 0)
								anglesTemp[distance<set<string>::const_iterator>((*it1).uniqueDOFlist.begin(), it2)].push_back(
										j * increments_.at(*it2) + minDOF_.at(*it2));
//						else
//								anglesTemp[distance<set<string>::const_iterator>((*it1).uniqueDOFlist.begin(), it2)].push_back(
//										(nbOfStep_ * increments_.at(*it2)) - (j * increments_.at(*it2)) + minDOF_.at(*it2));
					}
			for (int i = 0; i < pow(nbOfStep_, (maxIteration - iteration - 1)); i++) // column length
				for (int j = 0; j < nbOfStep_; j++)
					for (int k = 0; k < pow(nbOfStep_, iteration); k++)
					{
//						if(i%2 == 0)
							anglesBetweenNode[distance<set<string>::const_iterator>((*it1).uniqueDOFlist.begin(), it2)].push_back(
									j * increments_.at(*it2) + minDOF_.at(*it2) + increments_.at(*it2) / 2 );
//						else
//							anglesBetweenNode[distance<set<string>::const_iterator>((*it1).uniqueDOFlist.begin(), it2)].push_back(
//									j * increments_.at(*it2) + minDOF_.at(*it2) + increments_.at(*it2) / 2 );
					}

		}


//		cout << "set storage" << endl;
		(*it1).angleStorage.setName("AnglesData");
		(*it1).angleStorage.setInDegrees(false);
		(*it1).angleBetweenNodeStorage.setName("AnglesData");
		(*it1).angleBetweenNodeStorage.setInDegrees(false);


		// Add time colunm name.

		Array<string> array(String(), 0);
		array.insert(0, "time");
		for (set<string>::const_iterator it2 = (*it1).uniqueDOFlist.begin(); it2 != (*it1).uniqueDOFlist.end();it2++)
			array.append(*it2);

//		cout << "fill" << endl;
		(*it1).angleStorage.setColumnLabels(array);
		(*it1).angleBetweenNodeStorage.setColumnLabels(array);
		for (int i = 0; i < anglesTemp[0].size(); i++)
		{
			//double tabState[(*it1).uniqueDOFlist.size() - 1];
			//std::vector<double> tabState; //old
			SimTK::Vector_<double> tabState;

			tabState.resize((*it1).uniqueDOFlist.size());
			for (int j = 0; j < (*it1).uniqueDOFlist.size(); j++)
				tabState[j] = anglesTemp[j][i];
			//StateVector stateVector(i * 0.01, (*it1).uniqueDOFlist.size(), tabState.data()); // 10 ms times step //old
			StateVector stateVector(i * 0.01, tabState);
			(*it1).angleStorage.append(stateVector, false);
		}

		stringstream temp_str;
		temp_str<<(distance<vector<Task>::const_iterator>(taskVect_.begin(), it1));
		string temp = temp_str.str();
		if (printCEINMSData_)
		{
			boost::filesystem::path dir ( "cfg/OutputCalibration" );

			if ( !boost::filesystem::exists ( dir ) )
				if ( !boost::filesystem::create_directory ( dir ) )
					COUT << "ERROR in creating directory: " << "cfg/OutputCalibration" << std::endl;
			writeAngles((*it1).angleStorage, "cfg/OutputCalibration/angles_" + temp + ".in");
			writeAnglesBetweenNode(anglesBetweenNode, "cfg/OutputCalibration/angles_BetweenNode_" + temp + ".in", (*it1).angleBetweenNodeStorage);
		}

		if (printOsimData_)
		{
			boost::filesystem::path dir ( "cfg/OutputCalibration" );

			      if ( !boost::filesystem::exists ( dir ) )
				      if ( !boost::filesystem::create_directory ( dir ) )
					      COUT << "ERROR in creating directory: " << "cfg/OutputCalibration" << std::endl;
			(*it1).angleStorage.print("cfg/OutputCalibration/angles_" + temp + ".mot");
			(*it1).angleBetweenNodeStorage.print("cfg/OutputCalibration/angles_BetweenNode_" + temp + ".mot");
		}
	}

//	cout << "ending compute angles " << endl;

//	cout << "end compute angles" << endl;
//	printMemUsage();
}

void MuscleAnalyseForSpline::writeAngles(const Storage& storage, const string fileName)
{
	ofstream outputDataFile(fileName.c_str());
	outputDataFile << storage.getSize() << endl;
	for(int i = 0; i < storage.getSize(); i++)
	{
		//double temp[storage.getColumnLabels().getSize() - 2];
		std::vector<double> temp;
		temp.resize(storage.getColumnLabels().getSize() - 1);

		storage.getData(i, storage.getColumnLabels().getSize() - 1, temp.data());
		for(int j = 0; j < storage.getColumnLabels().getSize() - 1; j++) // -1 for the time
		{
			outputDataFile << std::fixed << std::setprecision(PRECISION) << temp[j] << "\t";
		}
		outputDataFile << endl;
	}
}

void MuscleAnalyseForSpline::writeAnglesBetweenNode(const vector<vector<double> >& data, const string fileName, Storage& storage)
{
	ofstream outputDataFile(fileName.c_str());
	outputDataFile << data[0].size() << endl;
	for(int i = 0; i < data[0].size(); i++)
	{
		//std::vector<double> tabState; //old
		SimTK::Vector_<double> tabState;

		tabState.resize(data.size());
		//double tabState[data.size()];
		for(int j = 0; j < data.size(); j++)
		{
			outputDataFile << std::fixed << std::setprecision(PRECISION) << data[j][i] << "\t";
			tabState[j] = data[j][i];
		}
		StateVector stateVector(i * 0.01, tabState);
		storage.append(stateVector);
		outputDataFile << endl;
	}
}

void MuscleAnalyseForSpline::run(bool betweenNodes)
{
//	cout << endl;
//	cout << "Beginning Muscle Analysis" << endl;
	for (vector<Task>::iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++)
	{
		si_ = osimModel_.initSystem();
//		cout << "Task: " << distance<vector<Task>::iterator>(taskVect_.begin(), it1) << endl;
//		printMemUsage();
		ptr_muscleAnalyse_ = boost::shared_ptr<MuscleAnalysis>(new MuscleAnalysis(&osimModel_));

		ptr_muscleAnalyse_->setComputeMoments(true); // For moment arm.

		Array<string> DOFNameArray;
		Array<string> muscleNameArray;
		vector<string> uniqueMuscleList;

		for(vector<string>::const_iterator it2 = (*it1).uniqueMuscleList.begin(); it2 != (*it1).uniqueMuscleList.end(); it2++)
			muscleNameArray.append(*it2);

		// If the muscle list doesnt match open sim muscle list. 
		if (muscleNameArray.size() != vectMuscleNameListOpenSim_.size())
			cout << "warning! Check Muscle Names in opensim and subject xml!" << endl;

		for(set<string>::const_iterator it2 = (*it1).uniqueDOFlist.begin(); it2 != (*it1).uniqueDOFlist.end(); it2++)
			DOFNameArray.append(*it2);

		ptr_muscleAnalyse_->setMuscles(muscleNameArray);
		ptr_muscleAnalyse_->setCoordinates(DOFNameArray);
		int NbOfLine;
		if(!betweenNodes)
		{
			ptr_muscleAnalyse_->setStatesStore((*it1).angleStorage);
			double data[1];
			(*it1).angleStorage.getData(1,1,data);
			NbOfLine = (*it1).angleStorage.getSize();
		}
		else
		{
			ptr_muscleAnalyse_->setStatesStore((*it1).angleBetweenNodeStorage);
			double data[1];
			(*it1).angleBetweenNodeStorage.getData(1,1,data);
			NbOfLine = (*it1).angleBetweenNodeStorage.getSize();
		}

		double t = 0.0;

		for (int i = 0; i < NbOfLine; i++)
		{
			(*it1).angleStorage.getTime(i, si_.updTime()); // time
			t = si_.getTime();
			setDOF(*it1, betweenNodes);
			osimModel_.getMultibodySystem().realize(si_, SimTK::Stage::Dynamics);
			if(i == 0)
				ptr_muscleAnalyse_->begin(si_);
			else if(i == NbOfLine)
				ptr_muscleAnalyse_->end(si_);
			else
				ptr_muscleAnalyse_->step(si_, i);
		}

//		cout << "save data" << endl;
//		printMemUsage();

		Storage* lmtStorage = ptr_muscleAnalyse_->getMuscleTendonLengthStorage();
		if (printOsimData_)
		{
			stringstream temp_str;
			temp_str<<(distance<vector<Task>::const_iterator>(taskVect_.begin(), it1));
			string temp = temp_str.str();
			if(!betweenNodes)
				lmtStorage->print("cfg/OutputCalibration/lmt_" + temp + ".mot");
			else
				lmtStorage->print("cfg/OutputCalibration/lmt_BetweenNode_" + temp + ".mot");
		}

		const ArrayPtrs<MuscleAnalysis::StorageCoordinatePair>& maArrayPtrs =
				ptr_muscleAnalyse_->getMomentArmStorageArray();
		const Array<std::string>& collumnLabels = lmtStorage->getColumnLabels();

		if(!betweenNodes)
			(*it1).lmtVectorMat.resize(collumnLabels.size() - 1);
		else
			(*it1).lmtVectorMatBetweenNode.resize(collumnLabels.size() - 1);

		for (int i = 0; i < collumnLabels.size() - 1; i++) // -1 for the time column
		{
			Array<double> colArray;
			lmtStorage->getDataColumn(i, colArray);
			for (int j = 0; j < colArray.size(); j++)
			{
				if(!betweenNodes)
					(*it1).lmtVectorMat[i].push_back(colArray[j]);
				else
					(*it1).lmtVectorMatBetweenNode[i].push_back(colArray[j]);
			}
		}

		if(!betweenNodes)
			(*it1).maVector3DMat.resize(maArrayPtrs.getSize());
		else
			(*it1).maVector3DMatBetweenNode.resize(maArrayPtrs.getSize());
		for (int i = 0; i < maArrayPtrs.getSize(); i++)
		{
			Storage* maTempStorage = maArrayPtrs[i]->momentArmStore;
			const Array<std::string>& collumnLabels = maTempStorage->getColumnLabels();
			const Coordinate* coord = maArrayPtrs[i]->q;
			const string& DOFName = coord->getName();
			if (printOsimData_)
			{
				stringstream temp_str;
				temp_str<<(distance<vector<Task>::const_iterator>(taskVect_.begin(), it1));
				string temp = temp_str.str();
				if(!betweenNodes)
					maTempStorage->print("cfg/OutputCalibration/ma_" + temp + "_" + DOFName + ".mot");
				else
					maTempStorage->print("cfg/OutputCalibration/ma_BetweenNode_" + temp + "_" + DOFName + ".mot");
			}
			if(!betweenNodes)
				(*it1).maVector3DMat[i].resize(collumnLabels.getSize() - 1);
			else
				(*it1).maVector3DMatBetweenNode[i].resize(collumnLabels.getSize() - 1);
			for (int j = 0; j < collumnLabels.getSize() - 1; j++)
			{
				Array<double> colArray;
				maTempStorage->getDataColumn(j, colArray);
				for (int k = 0; k < colArray.size(); k++)
				{
					if(!betweenNodes)
						(*it1).maVector3DMat[i][j].push_back(colArray[k]);
					else
						(*it1).maVector3DMatBetweenNode[i][j].push_back(colArray[k]);
				}
			}
		}
		ptr_muscleAnalyse_.reset();
	}
//	cout << "End of Muscle Analysis" << endl;
//	printMemUsage();
}

void MuscleAnalyseForSpline::setDOF(const Task& task, bool betweenNode)
{
	//double tabValue[task.uniqueDOFlist.size()];
	std::vector<double> tabValue;
	tabValue.resize(task.uniqueDOFlist.size());
	if(!betweenNode)
		task.angleStorage.getDataAtTime(si_.getTime(), task.uniqueDOFlist.size(), tabValue.data());
	else
		task.angleBetweenNodeStorage.getDataAtTime(si_.getTime(), task.uniqueDOFlist.size(), tabValue.data());
	for (set<string>::const_iterator it = task.uniqueDOFlist.begin(); it != task.uniqueDOFlist.end(); it++)
		osimModel_.updCoordinateSet().get(*it).setValue(si_, tabValue[distance<set<string>::const_iterator>(task.uniqueDOFlist.begin(), it)]);
	osimModel_.assemble(si_);
}

double MuscleAnalyseForSpline::getMinAnglesDof(const string& nameOfDOF) const
{
	const CoordinateSet& coordinateSet = osimModel_.getCoordinateSet();
	return coordinateSet.get(nameOfDOF).getRangeMin();
}

double MuscleAnalyseForSpline::getMaxAnglesDof(const string& nameOfDOF) const
{
	const CoordinateSet& coordinateSet = osimModel_.getCoordinateSet();
	return coordinateSet.get(nameOfDOF).getRangeMax();
}

void MuscleAnalyseForSpline::writeLmt() const
{
	if(printCEINMSData_)
	{
//		cout << endl;
//		cout << "Writing muscle tendon length in output directory..." << endl;

		for (vector<Task>::const_iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++)
		{
			stringstream temp_str;
			temp_str<<(distance<vector<Task>::const_iterator>(taskVect_.begin(), it1));
			string temp = temp_str.str();
			string outputDataFileName("cfg/OutputCalibration/lmt_" + temp + ".in");
			ofstream outputDataFile(outputDataFileName.c_str());
			outputDataFile << (*it1).lmtVectorMat[0].size() << endl;
			for (set<string>::const_iterator it2 = (*it1).uniqueDOFlist.begin();it2 != (*it1).uniqueDOFlist.end(); it2++)
				outputDataFile << *it2 << "\t";

			outputDataFile << endl;
			for (int i = 0; i < (*it1).lmtVectorMat[0].size(); i++)
			{
				for (int j = 0; j < (*it1).lmtVectorMat.size(); j++)
					outputDataFile << std::fixed << std::setprecision(PRECISION) << (*it1).lmtVectorMat[j][i] << "\t";
				outputDataFile << endl;
			}
			outputDataFile.close();

			string outputDataFileNameBN("cfg/OutputCalibration/lmt_BetweenNode_" + temp + ".in");
			ofstream outputDataFileBN(outputDataFileNameBN.c_str());
			outputDataFileBN << (*it1).lmtVectorMatBetweenNode[0].size() << endl;
			for (set<string>::const_iterator it2 = (*it1).uniqueDOFlist.begin();it2 != (*it1).uniqueDOFlist.end(); it2++)
				outputDataFileBN << *it2 << "\t";

			outputDataFileBN << endl;
			for (int i = 0; i < (*it1).lmtVectorMatBetweenNode[0].size(); i++)
			{
				for (int j = 0; j < (*it1).lmtVectorMatBetweenNode.size(); j++)
					outputDataFileBN << std::fixed << std::setprecision(PRECISION) << (*it1).lmtVectorMatBetweenNode[j][i] << "\t";
				outputDataFileBN << endl;
			}
			outputDataFileBN.close();
		}
	}

}

void MuscleAnalyseForSpline::writeMa() const
{
	if(printCEINMSData_)
	{
//		cout << endl;
//		cout << "Writing Muscle moment arm result in output directory..." << endl;

		typedef set<string>::const_iterator SetStrCstItDoF;
		typedef vector<string>::const_iterator SetStrCstItMuS;
		for (vector<Task>::const_iterator it1 = taskVect_.begin(); it1 != taskVect_.end(); it1++)
		{
			SetStrCstItDoF itDOF = (*it1).uniqueDOFlist.begin();
			for (int i = 0; i < (*it1).uniqueDOFlist.size(); i++)
			{
				stringstream temp_str;
				temp_str<<(distance<vector<Task>::const_iterator>(taskVect_.begin(), it1));
				string temp = temp_str.str();
				string outputDataFileName("cfg/OutputCalibration/ma_" + *itDOF + "_" + temp + ".in");
				ofstream outputDataFile(outputDataFileName.c_str());
				outputDataFile << (*it1).maVector3DMat[i][0].size() << endl;
				for (SetStrCstItMuS it2 = (*it1).uniqueMuscleList.begin();it2 != (*it1).uniqueMuscleList.end(); it2++)
						outputDataFile << (*it2) << "\t";
				outputDataFile << endl;
				for (int j = 0; j < (*it1).maVector3DMat[i][0].size(); j++)
				{
					for (int k = 0; k < (*it1).maVector3DMat[i].size(); k++)
						outputDataFile << std::fixed << std::setprecision(PRECISION) << (*it1).maVector3DMat[i][k][j]<< "\t";

					outputDataFile << endl;
				}
				outputDataFile.close();

				string outputDataFileNameBN("cfg/OutputCalibration/ma_BetweenNode_" + *itDOF + "_"  + temp + ".in");
				itDOF++;
				ofstream outputDataFileBN(outputDataFileNameBN.c_str());
				outputDataFileBN << (*it1).maVector3DMatBetweenNode[i][0].size() << endl;
				for (SetStrCstItMuS it2 = (*it1).uniqueMuscleList.begin();it2 != (*it1).uniqueMuscleList.end(); it2++)
						outputDataFileBN << (*it2) << "\t";
				outputDataFileBN << endl;
				for (int j = 0; j < (*it1).maVector3DMatBetweenNode[i][0].size(); j++)
				{
					for (int k = 0; k < (*it1).maVector3DMatBetweenNode[i].size(); k++)
						outputDataFileBN << std::fixed << std::setprecision(PRECISION) << (*it1).maVector3DMatBetweenNode[i][k][j]<< "\t";

					outputDataFileBN << endl;
				}
				outputDataFileBN.close();
			}
		}
	}
}

#endif
