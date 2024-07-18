template<typename NMSmodelT>
PagmoProblem<NMSmodelT>::PagmoProblem()
{
	//for (auto x : fp_)
	//	x = DBL_MAX;
	//std::cout << "default constructor called" << std::endl;
}

template<typename NMSmodelT>
PagmoProblem<NMSmodelT>::~PagmoProblem()
{
	//std::cout << "destructor called" << std::endl;
	//delete subject_;
	//delete staticComputation_;
	//delete Mutex_; //?

	/*for (int i = 0; i < numberOfThreads_; i++)
	{
		delete subject_.at(i);
		delete staticComputation_.at(i);
	}*/
}

template<typename NMSmodelT>
std::vector<double> PagmoProblem<NMSmodelT>::fitness(const std::vector<double>& dv) const
{
	//std::cout << "fitness called" << std::endl;
	std::vector<double> emgValues;
	//Mutex_->lock();
	subject_->getEmgs(emgValues);
	//std::cout << "subject_: " << subject_.at(0) << std::endl;
	//subject_.at(0)->getEmgs(emgValues);
	//std::cout << "getEmgs done" << std::endl;
	for (unsigned i = 0; i < muscleIndexWithEMGtoOptimize_.size(); ++i)
		emgValues.at(muscleIndexWithEMGtoOptimize_.at(i)) = dv.at(i);
	//std::cout << "emgValues set" << std::endl;
	//subject_.at(0)->setEmgs(emgValues);
	//std::cout << "setEmgs done" << std::endl;
	subject_->setEmgs(emgValues);
	//Mutex_->unlock();
	double fp = evalfp();
	//std::cout << fp << std::endl;
	return { fp };
	//evalfp(0,0);
	//return { fp_.at(0) };
}

/*template<typename NMSmodelT>
std::vector<double> PagmoProblem<NMSmodelT>::batch_fitness(const std::vector<double>& dvs) const
{
	//thread_ = new boost::thread(boost::bind(&OptimizationHybridPlugin<NMSmodelT>::optimization, this));
	//std::future<int> f = std::async(std::bind(&Worker::Do, &worker));
	//int retval = f.get(); //Will block until do returns an int.
	/*std::cout << "batch_fitness called" << std::endl;
	std::cout << "dvs size: " << dvs.size() << std::endl;
	std::cout << "dvs/numberOfThreads " << dvs.size() / numberOfThreads_ << std::endl;
	std::cout << "fp_ size:" << fp_.size() << std::endl;*/
	//std::vector<std::future<double>> fs;
	/*for (int populationIt = 0; populationIt < (dvs.size() / noParameters_); populationIt += numberOfThreads_) // populationIt: 0:#threads:100
	{
		{
			boost::thread_group threads;
			//std::cout << "thread_group size: " << threads.size() << std::endl;
			//std::cout << "populationIt: " << populationIt << std::endl;
			for (int iThread = 0; iThread < numberOfThreads_; iThread++) //iThread: 0:1:#threads
			{
				int iIndividual = populationIt + iThread; // iIndividual 0:1:100
				
				//std::cout << "i: " << i << std::endl;
				std::vector<double> emgValues;
				subject_.at(iThread)->getEmgs(emgValues);
				for (int iMuscle = 0; iMuscle < muscleIndexWithEMGtoOptimize_.size(); ++iMuscle)
				{
					emgValues.at(muscleIndexWithEMGtoOptimize_.at(iMuscle)) = dvs.at(iMuscle + iIndividual*4);
					//std::cout << "iMuscle + 4*i: " << iMuscle + 4*i + populationIt << std::endl;
				}
				subject_.at(iThread)->setEmgs(emgValues);

				//fs.at(i) = std::async( std::bind( &PagmoProblem<NMSmodelT>::evalfp, this) );
				//std::cout << "i: " << i << std::endl;
				//threads.create_thread(&evalfp(i));
				threads.create_thread(boost::bind(&PagmoProblem<NMSmodelT>::evalfp, this, iIndividual, iThread));
				
				//double fp = evalfp();
				//std::cout << "i: " << i << std::endl;
			}
			//std::cout << "thread_group size: " << threads.size() << std::endl;
			threads.join_all();
			/*for (auto it : fp_)
				std::cout << it << std::endl;*/
			/*//std::cout << "thread_group size: " << threads.size() << std::endl;
		}
		//std::cout << "threads joined" << std::endl;
	}
	//std::cout << "fp_ size:" << fp_.size() << std::endl;
	return fp_;
}*/

template<typename NMSmodelT>
std::pair<std::vector<double>, std::vector<double>> PagmoProblem<NMSmodelT>::get_bounds() const
{
	return bounds_;
}

template<typename NMSmodelT>
//void PagmoProblem<NMSmodelT>::setModel(const std::string configurationFile, unsigned numberOfThreads)
//void PagmoProblem<NMSmodelT>::setModel(const std::string configurationFile)
void PagmoProblem<NMSmodelT>::setModel(NMSmodelT* subject)
{
	//configurationFile_ = configurationFile;
	//numberOfThreads_ = numberOfThreads;
	//std::vector<std::string> dofName;
	//dofName.push_back("ankle_angle_l");
	//std::vector<unsigned int> dofsUsedDummy; // not used
	/*for (int i = 0; i < numberOfThreads_; i++)
	{
		subject_.push_back(new NMSmodelT());
		setupSubject(*subject_.at(i), configurationFile);
		subject_.at(i)->eraseUnusedDofs(dofName, dofsUsedDummy);
	}*/
	
	subject_ = subject;
	//subject_ = new NMSmodelT(); // new NMSmodelT(subject)
	
	//setupSubject(*subject_,configurationFile);
	//subject_->eraseUnusedDofs(dofName, dofsUsedDummy);
	subject_->getDoFNames(subjectDofNames_);

	//subject_.at(0)->getDoFNames(subjectDofNames_);
	externalTorques_.resize(subjectDofNames_.size()); 
	for (auto it = subjectDofNames_.begin(); it != subjectDofNames_.end(); ++it)
	{
		std::cout << *it << std::endl;
	}
	//staticComputation_.resize(numberOfThreads_);
	//fp_.resize(100); 
	//std::cout << "fp_ size:" << fp_.size() << std::endl;
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setMusclesNamesWithEmgToTrack(const std::vector<std::string>& musclesNamesWithEmgToTrack)
{
	musclesNamesWithEmgToTrack_.assign(musclesNamesWithEmgToTrack.begin(), musclesNamesWithEmgToTrack.end());
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setMusclesNamesWithEmgToPredict(const std::vector<std::string>& musclesNamesWithEmgToPredict)
{
	musclesNamesWithEmgToPredict_.assign(musclesNamesWithEmgToPredict.begin(), musclesNamesWithEmgToPredict.end());
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setParameters()
{
	/*parameters_ = new Parameters::RecursiveEMGs<NMSmodelT>(*subject_, musclesNamesWithEmgToTrack_, musclesNamesWithEmgToPredict_);
	std::vector<double> x;
	parameters_->getStartingVectorParameters(x); // x_ not used yet, but this is necessary to call setVectorParameters()*/

	//subject_.at(0)->getMusclesIndexFromMusclesList(muscleIndexWithEMGtoTrack_, musclesNamesWithEmgToTrack_);
	//subject_.at(0)->getMusclesIndexFromMusclesList(muscleIndexWithEMGtoPredict_, musclesNamesWithEmgToPredict_);
	subject_->getMusclesIndexFromMusclesList(muscleIndexWithEMGtoTrack_, musclesNamesWithEmgToTrack_);
	subject_->getMusclesIndexFromMusclesList(muscleIndexWithEMGtoPredict_, musclesNamesWithEmgToPredict_);

	//concatenate muscleIndexWithEMGtoTrack_ and muscleIndexWithEMGtoPredict_
	muscleIndexWithEMGtoOptimize_.assign(muscleIndexWithEMGtoTrack_.begin(), muscleIndexWithEMGtoTrack_.end());
	muscleIndexWithEMGtoOptimize_.insert(muscleIndexWithEMGtoOptimize_.end(), muscleIndexWithEMGtoPredict_.begin(), muscleIndexWithEMGtoPredict_.end());
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::set_bounds(std::vector<double> lowerBounds, std::vector<double> upperBounds)
{
	bounds_ = { lowerBounds, upperBounds };
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setMutex(boost::mutex& Mutex)
{
	Mutex_ = &Mutex;
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setSingleExternalTorque(double externalTorque, const std::string& whichDof)
{
	vector<string>::const_iterator it = subjectDofNames_.begin();
	while (*it != whichDof && it != subjectDofNames_.end())
		++it;
	/*std::cout << "*it: "<< *it << std::endl;
	std::cout << "whichDof: " << whichDof << std::endl;
	std::cout << "externalTorque: " << externalTorque << std::endl;*/
	if (*it == whichDof) {
		unsigned pos = std::distance<vector<string>::const_iterator>(subjectDofNames_.begin(), it);
		//std::cout << "pos: " << pos << std::endl;
		externalTorques_.at(pos) = externalTorque;
		//std::cout << "externalTorques_.at(pos): " << externalTorques_.at(pos) << std::endl;
		//currentDofNames_.at(pos) = whichDof;
	}
	else {
		std::cout << "ErrorMinimizer::setSingleExternalTorque ERROR\n" << whichDof << " not found in the subject\n";
		exit(EXIT_FAILURE);
	}
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setStaticComputation()
{
	/*for (int i = 0; i < numberOfThreads_; i++)
	{
		//std::cout << "staticComputation before deleting: " << staticComputation_ << std::endl;
		if (staticComputation_.at(i)) // should not be necessary if staticComputation is initialized as a null pointer
			delete staticComputation_.at(i);
	
		//std::cout << "before setting staticComputation_. subject_: " << subject_ << std::endl;         //*subject_
		staticComputation_.at(i) = new StaticComputation<NMSmodelT, StaticComputationMode::Default<NMSmodelT>>(*subject_.at(i), musclesNamesWithEmgToTrack_, musclesNamesWithEmgToPredict_);
		//std::cout << "after setting staticComputation_" << std::endl;
	}*/
	//std::cout << "staticComputation size: " << staticComputation_.size() << std::endl;

	if (staticComputation_) // should not be necessary if staticComputation is initialized as a null pointer
		delete staticComputation_;
	//std::cout << "before setting staticComputation_. subject_: " << subject_ << std::endl;
	staticComputation_ = new StaticComputation<NMSmodelT, StaticComputationMode::Default<NMSmodelT>>(*subject_, musclesNamesWithEmgToTrack_, musclesNamesWithEmgToPredict_);
	//std::cout << "after setting staticComputation_" << std::endl;
}

template<typename NMSmodelT>
void PagmoProblem<NMSmodelT>::setupSubject(NMSmodelT& mySubject, string configurationFile)
{
	SetupDataStructure<NMSmodelT, Curve<CurveMode::Online> > setupData(configurationFile);
	//std::cout << "configurationFile: " << configurationFile << std::endl;
	setupData.createCurves();
	//std::cout << "curves created" << std::endl;
	setupData.createMuscles(mySubject);
	//std::cout << "muscles created" << std::endl;
	setupData.createDoFs(mySubject);
	//std::cout << "DOFs created" << std::endl;
	setupData.createMusclesNamesOnChannel(mySubject);
	//std::cout << "channels created" << std::endl;
}

template<typename NMSmodelT>
//void PagmoProblem<NMSmodelT>::evalfp(int iIndividual, int iThread) const
double PagmoProblem<NMSmodelT>::evalfp() const
{
	//int i = 0;
	//std::cout << "iIndividual in evalfp: " << iIndividual << std::endl;
	//std::cout << "iThread in evalfp: " << iThread << std::endl;
	// 0) get data with mutex
	vector<double> torques;
	vector<double> experimentalEMGs, adjustedEMGs;
	vector<double> currentEMGs;
	//Mutex_->lock();
	staticComputation_->getTorques(torques);
	staticComputation_->getInitialValuesOfTrackedEMGs(experimentalEMGs); //emg value for the tracked muscles before the emg adjustment
	staticComputation_->getAdjustedValuesOfTrackedEMGs(adjustedEMGs);
	staticComputation_->getCurrentEMGs(currentEMGs);
	/*staticComputation_.at(iThread)->getTorques(torques);
	staticComputation_.at(iThread)->getInitialValuesOfTrackedEMGs(experimentalEMGs); //emg value for the tracked muscles before the emg adjustment
	staticComputation_.at(iThread)->getAdjustedValuesOfTrackedEMGs(adjustedEMGs);
	staticComputation_.at(iThread)->getCurrentEMGs(currentEMGs);*/
	//Mutex_->unlock();
	
	//1) calculate first term of objective function: Least Squares Fitting

	double torqueLeastSquaresFitting = .0;

	for (unsigned d = 0; d < torques.size(); ++d)
	{
		torqueLeastSquaresFitting += fabs(externalTorques_.at(d) - torques.at(d)) * fabs(externalTorques_.at(d) - torques.at(d));
		//std::cout << "externalTorques / torques: \t " << externalTorques.at(d) << " \t " << torques.at(d) << "\n";
	}
	//std::cout << "torque error: " << torqueLeastSquaresFitting << "\n";

	//2) calculate the second term of objective function: track experimental EMGs

	double emgTracking = .0;

	for (unsigned e = 0; e < adjustedEMGs.size(); ++e)
		emgTracking += fabs(experimentalEMGs.at(e) - adjustedEMGs.at(e));
	//std::cout << "EMG error: " << emgTracking << "\n";

	//3) Calculate the performance criterion term of the objective function
	double performanceCriterion = .0;
	//cout << "emgs in objective function \n";
	for (unsigned i = 0; i < currentEMGs.size(); ++i) {
		performanceCriterion += currentEMGs.at(i) * currentEMGs.at(i);
		//cout << currentEMGs.at(i) << " ";
	}
	//std::cout << "excitation" << " ";

	//std::cout << performanceCriterion << std::endl;
	
	double fp;
	fp = hybridParameters_.alpha * torqueLeastSquaresFitting + hybridParameters_.gamma * emgTracking + hybridParameters_.beta * performanceCriterion;
	//fp_.at(iIndividual) = hybridParameters_.alpha * torqueLeastSquaresFitting + hybridParameters_.gamma * emgTracking + hybridParameters_.beta * performanceCriterion;
	//std::cout << fp_.at(iIndividual) << std::endl;
	//cout << endl;
	//std::cout << "Excitations: " << emgSum << "\n";
	//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	return fp;
}

template<typename NMSmodelT>
std::ostream& operator<<(std::ostream& output, const PagmoProblem<NMSmodelT>& pagmoProblem)
{
	output << "performanceCriterion: " << pagmoProblem.performanceCriterion_ << std::endl;

	output << "musclesNamesWithEmgToTrack_:" << std::endl;
	for (auto it : pagmoProblem.musclesNamesWithEmgToTrack_)
		output << it << std::endl;

	output << "musclesNamesWithEmgToPredict_:" << std::endl;
	for (auto it : pagmoProblem.musclesNamesWithEmgToPredict_)
		output << it << std::endl;

	output << "subjectDofNames_:" << std::endl;
	for (auto it : pagmoProblem.subjectDofNames_)
		output << it << std::endl;

	output << "externalTorques_:" << std::endl;
	for (auto it : pagmoProblem.externalTorques_)
		output << it << std::endl;

	return output;
}