// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox".
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software,
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS.
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621,
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com

//TODO: create a class for spline lmt and ma computation

#include "TorquesComputationRT.h"
#include "TorquesComputation.h"

#include <iomanip>

//#define DEBUG
template<typename ComputationModeT, typename NMSmodelT>
TorquesComputationRT<ComputationModeT, NMSmodelT>::TorquesComputationRT ( NMSmodelT& subject,
		const std::string& inputDataDirectory, const std::vector<string>& idTrials,
		const std::vector<string>& dofsToCalibrate, const std::string& emgFileName,
		const std::string& configurationFile, const std::string& subjectName, const std::string& translateName,
		const std::vector<double>& cropMin, const std::vector<double>& cropMax, const bool& filterEMG, const double& emd) : TorquesComputation<ComputationModeT, NMSmodelT>::TorquesComputation ( subject )
{
// 	subject_ = subject ;
 	this->dofsToCalibrate_ = dofsToCalibrate ;
 	this->computationMode_ = ComputationModeT(subject) ;
	configurationFile_ = configurationFile ;
	subjectName_ = subjectName ;
	cropMin_ = cropMin ;
	cropMax_ = cropMax ;
	emd_ = emd;
	_emgFirstPass = true;
	
 	//COUT << subject.name << std::endl;

	//COUT << subjectName << std::endl;
	_verbose = 1;
	subject.getDoFNames ( dofNameVect_ );
	subject.getMuscleNames ( muscleNamesVect_ );
	
 	//COUT << "muscleNamesVect_.size(): "  << muscleNamesVect_.size() << std::endl;
	//COUT << "dofNameVect_.size(): " << dofNameVect_.size() << std::endl;
	
	// first we resize the vector that has the number of trials
	this->trials_.resize ( idTrials.size() );

	// and we set basic data
	for ( unsigned int i = 0; i < this->trials_.size(); ++i )
		this->trials_.at ( i ).id_ = idTrials.at ( i );
	
	try
	{
		std::auto_ptr<ExecutionEMGType> temp ( executionEMG ( emgFileName, xml_schema::flags::dont_initialize ) );
		executionEMGPointer_ = temp;
	}
	catch ( const xml_schema::exception& e )
	{
		COUT << e <<std::endl;
		exit ( EXIT_FAILURE );
	}
	
	subject.getMusclesIndexFromDofs ( this->musclesIndexList_, dofNameVect_ );

	MTUSplineDataRead* splineData = new MTUSplineDataRead ( this->configurationFile_, this->subjectName_ );
	splineData->readTaskCoefficients();
	this->taskVect_ = splineData->getTaskVect();
	
	for ( vector<string>::const_iterator it1 = this->dofNameVect_.begin(); it1 != dofNameVect_.end(); it1++ )
	{
		this->musclesNamesOnDof_.push_back ( splineData->getMuscleOnDof ( *it1 ) );
	}

	delete splineData;

	// For each trial
	for ( unsigned int i = 0; i < idTrials.size(); ++i )
	{

		// read LMT data
		string angletDataFilename = inputDataDirectory + "/" + idTrials.at ( i ) + "/ik.sto";

		if ( _verbose > 1 )
			cout << "Reading from: " << angletDataFilename << std::endl;
		
// 		COUT << "a" << std::endl;

		DataFromFile angleDataFromFile ( angletDataFilename );
		
// 		COUT << "b" << std::endl;

		this->trials_.at ( i ).noLmtSteps_ = angleDataFromFile.getNoTimeSteps();

		std::vector<VectD> angleData;

		VectD lmtData;
		std::vector<VectD> maData;
		angleData.resize ( angleDataFromFile.getColumnNames().size() );
		lmtData.resize ( this->muscleNamesVect_.size() );
		maData.resize ( this->muscleNamesVect_.size() );
		const std::vector<string> dofNameVectFile = angleDataFromFile.getColumnNames();

		for ( int j = 0; j < this->musclesNamesOnDof_.size(); j++ )
		{
			maData[j].resize ( this->musclesNamesOnDof_[j].size() );
		}



		double startLmtTime = 0;

		for ( int j = 0; j < this->trials_.at ( i ).noLmtSteps_; ++j )
		{
			angleDataFromFile.readNextData();
			double lmtDataTime = angleDataFromFile.getCurrentTime();
			this->trials_.at ( i ).lmtTimeSteps_.push_back ( lmtDataTime );
			const VectD& angleDataTemp = angleDataFromFile.getCurrentData();

			for ( VectD::const_iterator it = angleDataTemp.begin(); it != angleDataTemp.end(); it++ )
				angleData[std::distance<VectD::const_iterator> ( angleDataTemp.begin(), it )].push_back ( *it );
		}

		// now, for each DoF... let us read the ma for the different muscles
		this->trials_.at ( i ).dofNames_ = dofNameVect_;
		this->trials_.at ( i ).noDoF_ = (int) this->trials_.at ( i ).dofNames_.size();
		this->trials_.at ( i ).maData_.resize ( this->muscleNamesVect_.size() );

		for ( int cpt = 0; cpt < angleData.back().size(); cpt++ )
		{
			
			for ( std::vector<MTUSplineDataRead::Task>::iterator it1 = this->taskVect_.begin(); it1 != taskVect_.end(); it1++ )
			{
				std::vector<double> lmt;
				std::vector<std::vector<double> > ma;
				std::vector<double> angle;

				for ( std::set<string>::const_iterator it2 = it1->uniqueDOFlist.begin(); it2 != it1->uniqueDOFlist.end();
						it2++ )
					for ( vector<string>::const_iterator it3 = dofNameVectFile.begin(); it3 != dofNameVectFile.end();
							it3++ )
					{
						if (  *it2  == *it3 )
						{
							angle.push_back (
								angleData[distance < vector < string > ::const_iterator > ( dofNameVectFile.begin(), it3 )][cpt] );
							break;
						}
						/*else //if(dofNameVectFile.end() == it3)
						{
							COUT << *it3 << " = " << *it2 << std::endl;
						}*/
					}

				this->noMuscles_ = (int) it1->uniqueMuscleList.size();

				switch ( ( *it1 ).uniqueDOFlist.size() )
				{
					case 1:
					{
						vector<std::shared_ptr<MTUSpline<1> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<1> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, angle, lmt, ma );
						break;
					}

					case 2:
					{
						vector<std::shared_ptr<MTUSpline<2> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<2> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 2, angle, lmt, ma );
						break;
					}

					case 3:
					{
						vector<std::shared_ptr<MTUSpline<3> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<3> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 3, angle, lmt, ma );
						break;
					}

					case 4:
					{
						vector<std::shared_ptr<MTUSpline<4> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<4> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 4, angle, lmt, ma );
						break;
					}

					case 5:
					{
						vector<std::shared_ptr<MTUSpline<5> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<5> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 5, angle, lmt, ma );
						break;
					}

					case 6:
					{
						vector<std::shared_ptr<MTUSpline<6> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<6> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 6, angle, lmt, ma );
						break;
					}

					case 7:
					{
						vector<std::shared_ptr<MTUSpline<7> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<7> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 7, angle, lmt, ma );
						break;
					}

					case 8:
					{
						vector<std::shared_ptr<MTUSpline<8> > > temp;
						temp.resize ( ( *it1 ).splines_.size() );

						for ( int i = 0; i < ( *it1 ).splines_.size(); i++ )
							temp[i] = std::dynamic_pointer_cast<MTUSpline<8> > ( ( *it1 ).splines_[i] );

						computeLmtMafromSplines ( temp, 8, angle, lmt, ma );
						break;
					}

					default:
						cout << "The dimension is bigger than 8 which is the maximum accepted." <<std::endl;
						exit ( 1 );
				}

				for ( vector<string>::const_iterator it2 = this->muscleNamesVect_.begin(); it2 != this->muscleNamesVect_.end();
						it2++ )
				{
					// std::vector<string>::iterator it3 = it1->uniqueMuscleList.find ( *it2 );
					vector<string>::iterator it3 = find(it1->uniqueMuscleList.begin(), it1->uniqueMuscleList.end(), *it2);

					if ( it3 != it1->uniqueMuscleList.end() )
					{
						lmtData[std::distance<vector<string>::const_iterator> ( this->muscleNamesVect_.begin(), it2 )] =
							lmt[std::distance<std::vector<string>::iterator> ( it1->uniqueMuscleList.begin(), it3 )];
					}
				}

				for ( std::vector<std::vector<std::string> >::const_iterator it2 = this->musclesNamesOnDof_.begin();
						it2 != this->musclesNamesOnDof_.end(); it2++ )
				{
					std::set<string>::iterator it4 = it1->uniqueDOFlist.find (
							this->dofNameVect_[distance<std::vector<std::vector<std::string> >::const_iterator> (
									this->musclesNamesOnDof_.begin(), it2 )] );

					if ( it4 != it1->uniqueDOFlist.end() )
					{
						for ( std::vector<std::string>::const_iterator it3 = it2->begin(); it3 != it2->end(); it3++ )
						{

							//std::vector<std::string>::iterator it5 = it1->uniqueMuscleList.find ( *it3 );
							std::vector<std::string>::iterator it5 = find(it1->uniqueMuscleList.begin(), it1->uniqueMuscleList.end(), *it3);

							if ( it5 != it1->uniqueMuscleList.end() )
							{

								maData[std::distance<std::vector<std::vector<std::string> >::const_iterator> (
										musclesNamesOnDof_.begin(), it2 )][std::distance <
												std::vector<std::string>::const_iterator > ( it2->begin(), it3 )] =
														ma[std::distance<std::set<std::string>::iterator> ( it1->uniqueDOFlist.begin(),
																it4 )][std::distance<std::vector<std::string>::iterator> (
																		it1->uniqueMuscleList.begin(), it5 )];
							}
						}
					}
				}
			}

			this->trials_.at ( i ).lmtData_.push_back ( lmtData );

			for ( int j = 0; j < this->muscleNamesVect_.size(); j++ )
			{
				this->trials_.at ( i ).maData_[j].push_back ( maData[j] );
			}
		}
		
		if( this->trials_.at ( i ).lmtData_.at(0).size() == 0 )
		{
			COUT << "Something is wrong for the " << idTrials.at(i) << "/ik.sto, the numbers of row is 0. Please check your files like the numbers of row, numbers of column or the name of the DOF." << std::endl;
			exit(EXIT_FAILURE);
		}

		// read TORQUE data
		this->trials_.at ( i ).torqueData_.resize ( this->trials_.at ( i ).noDoF_ );

		string momentDataFilename = inputDataDirectory + "/" + idTrials.at ( i ) + "/id.sto";

		if ( _verbose > 1 )
			COUT << "Reading from: " << momentDataFilename << std::endl;

		DataFromFile momentDataFromFile ( momentDataFilename );

		std::vector<VectD> momentData;
		momentData.resize ( momentDataFromFile.getColumnNames().size());

		this->trials_.at ( i ).noTorqueSteps_ = momentDataFromFile.getNoTimeSteps();
		
// 		COUT << this->trials_.at ( i ).noTorqueSteps_ << std::endl << std::flush;

		int cptLmt = 0;

		for ( int j = 0; j < (int) this->trials_.at ( i ).noTorqueSteps_; ++j )
		{
			momentDataFromFile.readNextData();
			this->trials_.at ( i ).torqueTimeSteps_.push_back ( momentDataFromFile.getCurrentTime() );

			const VectD& momentDataTemp = momentDataFromFile.getCurrentData();

			cptLmt++;

			for ( VectD::const_iterator it = momentDataTemp.begin(); it != momentDataTemp.end(); it++ )
				momentData.at ( std::distance<VectD::const_iterator> ( momentDataTemp.begin(), it ) ).push_back ( *it );
		}
		
// 		COUT << momentData.at(0).size() << std::endl << std::flush;
// 		COUT << this->dofNameVect_.size() << std::endl << std::flush;

		for ( int j = 0; j < (int) this->trials_.at ( i ).noTorqueSteps_; ++j )
		{
			for ( vector<string>::const_iterator it1 = this->dofNameVect_.begin(); it1 != this->dofNameVect_.end(); it1++ )
			{
				for ( vector<string>::const_iterator it2 = momentDataFromFile.getColumnNames().begin();
						it2 != momentDataFromFile.getColumnNames().end(); it2++ )
				{
					if (*it1 == *it2 || *it1 + "_moment" == *it2) // Added moment for Opensim Files
					{
						this->trials_.at ( i ).torqueData_.at (
							std::distance<vector<string>::const_iterator> ( this->dofNameVect_.begin(), it1 ) ).push_back ( momentData.at (
									std::distance<vector<string>::const_iterator> ( momentDataFromFile.getColumnNames().begin(), it2 ) ).at ( j ) );
					}
				}
			}
		}
		
		if( this->trials_.at ( i ).torqueData_.at(0).size() == 0 )
		{
			COUT << "Something is wrong for the " << idTrials.at(i) << "/id.sto, the numbers of row is 0. Please check your files like the numbers of row, numbers of column or the name of the DOF." << std::endl;
			exit(EXIT_FAILURE);
		}
		this->trials_.at ( i )._timeInit = this->trials_.at ( i ).torqueTimeSteps_[0];

		this->trials_.at ( i ).crop ( this->cropMin_.at ( i ), this->cropMax_.at ( i ) );
		
		std::string EMGDataFilename;
		
		// read EMG data
		if(filterEMG)
		  EMGDataFilename = inputDataDirectory + "/" + idTrials.at ( i ) + "/emg.sto";
		else
		  EMGDataFilename = inputDataDirectory + "/" + idTrials.at ( i ) + "/emgFilt.sto";

		
			cout << "Reading from: " << EMGDataFilename << std::endl;

		initEMG ( i, EMGDataFilename, filterEMG );

	} // done with the trials

	//initialize the computation mode
	this->computationMode_.setTrials ( this->trials_ );
	if (filterEMG)
	{
		ExecutionEmgXml xmlEmg(emgFileName);
		xmlEmg.setMaxEmg(_maxEMG);
		xmlEmg.UpdateEmgXmlFile();
	}
}

std::vector<double> selectEMGChannels(const std::vector<double>& emgDataFromFile, const std::vector<std::string>& channelNamesOnEMGFile, const std::vector<std::string>& channelNamesOnSubject){
	std::vector<double> selectedChannels;

	for(const auto& channelName : channelNamesOnSubject){
		int idx = std::distance(channelNamesOnEMGFile.begin(), find(channelNamesOnEMGFile.begin(), channelNamesOnEMGFile.end(), channelName));
		selectedChannels.push_back(emgDataFromFile[idx]);

	}

	return selectedChannels;

}



template<typename ComputationModeT, typename NMSmodelT>
void TorquesComputationRT<ComputationModeT, NMSmodelT>::initEMG ( const int& i, const string& EMGDataFilename, bool useFilter )
{
	const std::vector<double>& aCoeffHP = this->executionEMGPointer_->hpFilter().aCoeff();
	const std::vector<double>& bCoeffHP = this->executionEMGPointer_->hpFilter().bCoeff();
	const std::vector<double>& aCoeffLP = this->executionEMGPointer_->lpFilter().aCoeff();
	const std::vector<double>& bCoeffLP = this->executionEMGPointer_->lpFilter().bCoeff();
	const std::vector<double>& aCoeffDC = this->executionEMGPointer_->dcFilter().aCoeff();
	const std::vector<double>& bCoeffDC = this->executionEMGPointer_->dcFilter().bCoeff();
	const std::vector<double>& maxAmp = this->executionEMGPointer_->maxEMG();

	MapSS musclesNamesOnChannel;
	MapSS channelNameOnMuscleTemp;
	UnMapSS channelNameOnMuscle;
	std::map<string, double> mapData;
	std::vector<VectD> emgData;
	std::map<string, double> mapMuscle;
	std::vector<EMGPreProcessing*> emgPreProcessingVect;
	this->subject_.getMusclesNamesOnChannel ( musclesNamesOnChannel );
	emgData.resize ( musclesNamesOnChannel.size() );

	if ( _emgFirstPass )
	{
		_maxEMG.resize ( musclesNamesOnChannel.size(), 0 );
		_emgFirstPass = false;
	}

	for ( MapSS::const_iterator it1 = musclesNamesOnChannel.begin(); it1 != musclesNamesOnChannel.end(); it1++ )
		for ( vector<string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++ )
			channelNameOnMuscleTemp[*it2].push_back ( it1->first );

	for ( vector<string>::const_iterator it1 = this->muscleNamesVect_.begin(); it1 != this->muscleNamesVect_.end(); it1++ )
	{
		try
		{
			//This piece of code seems useless. channelNameOnMuscle is uninitialized
			channelNameOnMuscle[*it1] = channelNameOnMuscleTemp.at ( *it1 );
		}
		catch ( const std::out_of_range& /*oor*/ )
		{
			std::cout << "Channel not found for muscle " << *it1 << ". Replaced with 0 EMG." <<std::endl;
			continue; 
		}
	}
	
	DataFromFile emgDataFromFile ( EMGDataFilename );
	std::vector<std::string> channelNamesOnSubject;
	std::vector<std::string> channelNamesOnEMGFile = emgDataFromFile.getColumnNames();

	
	//Checks if channels on subject are present on EMG file, instead of checking number of channels. The selection of the proper channels is already done below.

	for(const auto& channelPair : musclesNamesOnChannel){
		channelNamesOnSubject.push_back(channelPair.first);
		if(std::find(channelNamesOnEMGFile.begin(), channelNamesOnEMGFile.end(), channelPair.first) == channelNamesOnEMGFile.end()){
			std::cout << "EMG channel " << channelPair.first << " not found in emg file" << std::endl;
			exit ( 1 );
		}
	}

	if(useFilter)
		for ( MapSS::const_iterator it = musclesNamesOnChannel.begin(); it != musclesNamesOnChannel.end(); it++ )
		{
			emgPreProcessingVect.push_back (
				new EMGPreProcessing ( aCoeffLP, bCoeffLP, aCoeffHP, bCoeffHP,
						maxAmp[std::distance<MapSS::const_iterator> ( musclesNamesOnChannel.begin(), it )] ) );
		}

	this->trials_.at ( i ).noMuscles_ = (int) this->muscleNamesVect_.size();
	this->trials_.at ( i ).noEmgSteps_ = (int) emgDataFromFile.getNoTimeSteps();



	/**
	 * @TODO:
	 * find ways to reduce computation time maybe crop earlier.
	 */

	while ( emgDataFromFile.areStillData() )
	{
		double EMGDataTime;
		emgDataFromFile.readNextData();
		EMGDataTime = emgDataFromFile.getCurrentTime();
		this->trials_.at ( i ).emgTimeSteps_.push_back ( EMGDataTime );

		// Filters channels relevat to subject only
		const VectD emgTemp = selectEMGChannels(emgDataFromFile.getCurrentData(), channelNamesOnEMGFile, channelNamesOnSubject);



		for ( VectD::const_iterator it = emgTemp.begin(); it != emgTemp.end(); it++ )
		{
//			std::cout << std::distance<VectD::const_iterator>(emgTemp.begin(), it) << std::endl;
			emgData[std::distance<VectD::const_iterator> ( emgTemp.begin(), it )].push_back ( *it );
		}
	}

	if(useFilter)
	{
		for ( std::vector<VectD>::iterator it = emgData.begin(); it != emgData.end(); it++ )
		{
			const int& cpt = std::distance<std::vector<VectD>::iterator> ( emgData.begin(), it );

			if ( _verbose > 1 )
				COUT << " Channel: " << emgDataFromFile.getColumnNames() [cpt];

			for ( VectD::const_iterator it2 = it->begin(); it2 != it->end(); it2++ )
				emgPreProcessingVect[cpt]->computeData ( *it2 );


			const double& maxEmg = emgPreProcessingVect[cpt]->getMax();

			if ( _maxEMG[cpt] < maxEmg )
			{
				_maxEMG[cpt] = maxEmg;
			}

			if ( _verbose > 1 )
				COUT << " MaxEMG: " << _maxEMG[cpt] << " ";

			delete emgPreProcessingVect[cpt];
			emgPreProcessingVect[cpt] = new EMGPreProcessing ( aCoeffLP, bCoeffLP, aCoeffHP, bCoeffHP,
					_maxEMG[cpt] );

			for ( VectD::iterator it2 = it->begin(); it2 != it->end(); it2++ )
				*it2 = emgPreProcessingVect[cpt]->computeData ( *it2 );
		}

		if ( _verbose > 1 )
			std::cout << std::endl;
	}


	bool firstPass = true;
	unsigned startSampleEMG = 0;
	unsigned stopSampleEMG = 0;

	for ( vector<double>::const_iterator it = this->trials_.at ( i ).emgTimeSteps_.begin();
			it != this->trials_.at ( i ).emgTimeSteps_.end(); it++ )
	{
		if ((this->cropMin_.at(i) + this->trials_.at(i)._timeInit) <= *it + emd_ && firstPass)
		{
			startSampleEMG = std::distance <
					vector<double>::const_iterator > (
							this->trials_.at ( i ).emgTimeSteps_.begin(), it );

			firstPass = false;
		}

		if ((this->cropMax_.at(i) + this->trials_.at(i)._timeInit) < *it + emd_)
		{
			stopSampleEMG = std::distance <
					vector<double>::const_iterator > (
							this->trials_.at ( i ).emgTimeSteps_.begin(), it );

			break;
		}
	}


// 	this->trials_.at ( i )._timeInit = this->trials_.at ( i ).emgTimeSteps_[0];

	for ( vector<VectD >::iterator itEMG = emgData.begin(); itEMG < emgData.end(); itEMG++ )
	{
		VectD::const_iterator cropStart = itEMG->begin()
				+ startSampleEMG;

		VectD::const_iterator cropEnd = itEMG->begin()
				+ stopSampleEMG;

// 		if ( cropEnd == itEMG->begin() )
// 		{
// 			stopSampleEMG = this->trials_.at ( i ).emgTimeSteps_.size();
// 			cropEnd = itEMG->end();
// 		}
				
// 				COUT << startSampleEMG << " : " << stopSampleEMG << std::endl;
				
		if(startSampleEMG >= stopSampleEMG)
		{
			COUT << "In EMG file for trial: " << this->trials_.at ( i ).id_ << " Cropping time is incorrect. Time bondary out of the time in the file." << std::endl << std::flush;
			exit(EXIT_FAILURE);
		}

		VectD temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			COUT << "EMG cropping failure\n";
			return;
		}

		*itEMG = temp;
	}
	
// 	COUT << startSampleEMG << " : " << stopSampleEMG << std::endl;

	{
		if(startSampleEMG == stopSampleEMG)
		{
			COUT << "In EMG file for trial: " << this->trials_.at ( i ).id_ << " Cropping time is incorrect. Time out of the time in the file." << std::endl << std::flush;
			exit(EXIT_FAILURE);
		}
	  
		vector<double>::const_iterator cropStart = this->trials_.at ( i ).emgTimeSteps_.begin()
				+ startSampleEMG;
		vector<double>::const_iterator cropEnd = this->trials_.at ( i ).emgTimeSteps_.begin()
				+ stopSampleEMG;
		vector<double> temp ( cropStart, cropEnd );

		if ( temp.size() == 0 )
		{
			COUT << "EMG cropping failure\n";
			return;
		}
		
		for (vector<double>::iterator itTime = temp.begin(); itTime != temp.end(); itTime++)
		{
			*itTime = *itTime - emd_;
		}
		
		this->trials_.at ( i ).emgTimeSteps_ = temp;
	}
	


	this->trials_.at ( i ).noEmgSteps_ = (int) this->trials_.at ( i ).emgTimeSteps_.size();

	// Channels relevant for subject were already selected
	for ( VectD::const_iterator it1 = emgData.back().begin(); it1 != emgData.back().end(); it1++ )
	{
		VectD tempEMG;
		const int cpt = std::distance<VectD::const_iterator> ( emgData.back().begin(), it1 );

		for ( std::vector<std::string>::const_iterator it = channelNamesOnSubject.begin();
				it != channelNamesOnSubject.end(); it++ )
		{
			mapData[*it] = emgData[std::distance<std::vector<std::string>::const_iterator> (
					channelNamesOnSubject.begin(), it )][cpt];
		}

		for ( UnMapSS::const_iterator it1 = channelNameOnMuscle.begin(); it1 != channelNameOnMuscle.end(); it1++ )
		{
			double data = 0;

			for ( vector<string>::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); it2++ )
			{
// 				COUT << *it2 << std::endl<< std::flush;
				if (mapData.find(*it2) != mapData.end())
					data += mapData.at(*it2);
				else
					data += 0; // this can be dangerous if emg name is not found it put it to zero
			}

			data /= it1->second.size();
			mapMuscle[it1->first] = data;
		}

		for ( std::vector<std::string>::const_iterator it2 = this->muscleNamesVect_.begin(); it2 != this->muscleNamesVect_.end(); it2++ )
		{
			if (mapMuscle.find(*it2) != mapMuscle.end())
			{
				//std::cout << *it2 << " : " << mapMuscle.at(*it2) << std::endl;
				tempEMG.push_back(mapMuscle.at(*it2));
			}
			else
				tempEMG.push_back ( 0 );
		}

		this->trials_.at ( i ).emgData_.push_back ( tempEMG );
	}
	
	if( this->trials_.at ( i ).emgData_.at(0).size() == 0 )
		{
			COUT << "Something is wrong for the " << this->trials_.at ( i ).id_ << " emg file, the numbers of row is 0. Please check your files like the numbers of row, numbers of column or the name of the channel." << std::endl;
			exit(EXIT_FAILURE);
		}
	
// 	for(std::vector<std::vector<double> >::const_iterator it = this->trials_.at ( i ).emgData_.begin(); it != this->trials_.at ( i ).emgData_.end(); it++ )
// 	  std::cout << it->at(0) << std::endl;

	if(useFilter)
		for ( std::vector<EMGPreProcessing*>::iterator it = emgPreProcessingVect.begin(); it != emgPreProcessingVect.end(); it++ )
			delete *it;
	
}

template<typename ComputationModeT, typename NMSmodelT>
void TorquesComputationRT<ComputationModeT, NMSmodelT>::setTimeTorques ( std::vector<std::vector<double> >& TimeTorques )
{
	TimeTorques.resize ( this->trials_.size() );

	for ( unsigned int i = 0; i < this->trials_.size(); ++i )
	{
		TimeTorques.at ( i ) = this->trials_.at ( i ).lmtTimeSteps_;
	}
}

template<typename ComputationModeT, typename NMSmodelT>
template<class T>
void TorquesComputationRT<ComputationModeT, NMSmodelT>::computeLmtMafromSplines ( T& splines, int dim,
		const std::vector<double>& angles, std::vector<double>& lmt, std::vector<std::vector<double> >& ma )
{
	lmt.clear();
	ma.clear();

	for ( int i = 0; i < this->noMuscles_; ++i )
		lmt.push_back ( splines[i]->getValue ( angles ) );

	ma.resize ( dim );

	for ( int k = 0; k < dim; ++k )
		for ( int i = 0; i < this->noMuscles_; ++i )
			ma[k].push_back ( -splines[i]->getFirstDerivative ( angles, k ) );
}

template<typename ComputationModeT, typename NMSmodelT>
void TorquesComputationRT<ComputationModeT, NMSmodelT>::computeLmtMafromSplines (
	std::vector<std::shared_ptr<MTUSpline<1> > >& splines, const std::vector<double>& angles,
	std::vector<double>& lmt, std::vector<std::vector<double> >& ma )
{
	lmt.clear();
	ma.clear();

	for ( int i = 0; i < this->noMuscles_; ++i )
		lmt.push_back ( splines[i]->getValue ( angles[0] ) );

	ma.resize ( 1 );

	for ( int i = 0; i < this->noMuscles_; ++i )
		ma[0].push_back ( -splines[i]->getFirstDerivative ( angles[0] ) );
}
