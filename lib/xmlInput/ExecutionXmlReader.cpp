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
#include "ExecutionXmlReader.h"


#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::endl;

ExecutionXmlReader::ExecutionXmlReader ( const string& filename )
	: runMode_ ( 0 ), isRealTime_ ( false )
{

	try
	{
		std::auto_ptr<ExecutionType> executionPointer ( execution ( filename, xml_schema::flags::dont_initialize ) );
		executionPointer_ = executionPointer;
	}
	catch ( const xml_schema::exception& e )
	{
		cout << e << endl << std::flush;
		exit ( EXIT_FAILURE );
	}

	readXml();
}

void ExecutionXmlReader::readXml()
{

	try
	{
		ExecutionType::NMSmodel_type& myModel ( executionPointer_->NMSmodel() );
		ExecutionType::NMSmodel_type::activation_type& myActivation ( myModel.activation() );
		ExecutionType::NMSmodel_type::activation_type::exponential_optional& myExpOpt ( myActivation.exponential() );
		ExecutionType::NMSmodel_type::activation_type::piecewise_optional& myPWOpt ( myActivation.piecewise() );

		if ( myExpOpt.present() )
			runMode_ += NMSModelCfg::ExponentialActivation;
		else if ( myPWOpt.present() )
			runMode_ += NMSModelCfg::PiecewiseActivation;
		else
		{
			cout << "invalid XML: ActivationType not found\n" << std::flush;
			exit ( EXIT_FAILURE );
		}

		ExecutionType::NMSmodel_type::tendon_type& myTendon ( myModel.tendon() );
		ExecutionType::NMSmodel_type::tendon_type::stiff_optional& myStiffOpt ( myTendon.stiff() );
		ExecutionType::NMSmodel_type::tendon_type::elastic_optional& myElsaticOpt ( myTendon.elastic() );
		ExecutionType::NMSmodel_type::tendon_type::elasticBiSec_optional& myElsaticBiSecOpt ( myTendon.elasticBiSec() );

		if ( myStiffOpt.present() )
			runMode_ += NMSModelCfg::StiffTendon;
		else if ( myElsaticOpt.present() )
			runMode_ += NMSModelCfg::ElasticTendon;
		else if ( myElsaticBiSecOpt.present() )
			runMode_ += NMSModelCfg::ElasticTendonBiSec;
		else
		{
			cout << "invalid XML: TendonType not found\n" << std::flush;
			exit ( EXIT_FAILURE );
		}

		ExecutionType::NMSmodel_type::type_type& myType ( myModel.type() );
		ExecutionType::NMSmodel_type::type_type::hybrid_optional& myHybOpt ( myType.hybrid() );
		ExecutionType::NMSmodel_type::type_type::openLoop_optional& myOLOpt ( myType.openLoop() );
		ExecutionType::NMSmodel_type::type_type::realTime_optional& myRTOpt ( myType.realTime() );

		if ( myHybOpt.present() )
			runMode_ += NMSModelCfg::Hybrid;
		else if ( myOLOpt.present() )
			runMode_ += NMSModelCfg::OpenLoop;
		else if ( myRTOpt.present() )
		{
			isRealTime_ = true;
			runMode_ += NMSModelCfg::RealTime;
			ExecutionType::NMSmodel_type::type_type::hybrid_optional& myRTHybOpt ( myType.realTime()->hybrid() );
			ExecutionType::NMSmodel_type::type_type::openLoop_optional& myRTOLOpt ( myType.realTime()->openLoop() );

			if ( myRTHybOpt.present() )
				runMode_ += NMSModelCfg::Hybrid;
			else if ( myRTOLOpt.present() )
				runMode_ += NMSModelCfg::OpenLoop;
		}
		else
		{
			cout << "invalid XML: Hybrid/OpenLoop/realTime Type not found\n" << std::flush;
			exit ( EXIT_FAILURE );
		}

		ExecutionType::NMSmodel_type::curve_type::online_optional& myOnlineOpt ( myModel.curve().online() );
		ExecutionType::NMSmodel_type::curve_type::offline_optional& myOfflineOpt ( myModel.curve().offline() );

		if ( myOnlineOpt.present() )
			runMode_ += NMSModelCfg::Online;
		else if ( myOfflineOpt.present() )
			runMode_ += NMSModelCfg::Offline;
		else
		{
			cout << "invalid XML: Online/Offline Type not found\n" << std::flush;
			exit ( EXIT_FAILURE );
		}

//         _nameOfSubject = executionPointer_->NameOfSubject();

// 		if ( executionPointer_->ConsumerPlugin().AngleDeviceFile().present() )
// 			_anglePlugin = executionPointer_->ConsumerPlugin().AngleDeviceFile();
//
// 		if ( executionPointer_->ConsumerPlugin().EMGDeviceFile().present() )
// 			_emgPlugin = executionPointer_->ConsumerPlugin().EMGDeviceFile();
//
// 		if ( executionPointer_->ConsumerPlugin().ComsumerFile().present() )
// 			_emgPlugin = executionPointer_->ConsumerPlugin().ComsumerFile();
//
// 		if ( executionPointer_->ConsumerPlugin().AngleAndComsumerDevice().present() )
// 			_emgPlugin = executionPointer_->ConsumerPlugin().AngleAndComsumerDevice();


//ANCORA DA IMPLEMENTARE
		/*
		       ExecutionType::samplingFrequency_optional& myFreqOpt(executionPointer->samplingFrequency());
		       if(myFreqOpt.present())
		           myExecutionCfg.frequency_ = ExecutionCfg::Enabled;
		       else
		           myExecutionCfg.frequency_ = ExecutionCfg::Disabled;

		       ExecutionType::logging_type::txt_optional& myTxtOpt(executionPointer->logging().txt());
		       ExecutionType::logging_type::csv_optional& myCsvOpt(executionPointer->logging().csv());
		       ExecutionType::logging_type::mot_optional& myMotOpt(executionPointer->logging().mot());
		       if(myTxtOpt.present())
		           myExecutionCfg.logging_ = ExecutionCfg::txt;
		       else if(myCsvOpt.present())
		           myExecutionCfg.logging_ = ExecutionCfg::csv;
		       else if(myMotOpt.present())
		           myExecutionCfg.logging_ = ExecutionCfg::mot;
		       else {
		           cout << "invalid XML: Logging Type not found\n";
		           exit(EXIT_FAILURE);
		       }
		*/
	}
	catch ( const xml_schema::exception& e )
	{
		cout << e << endl;
		exit ( EXIT_FAILURE );
	}


}

std::string ExecutionXmlReader::getNameOfSubject()
{
	return executionPointer_->NameOfSubject();
}

std::string ExecutionXmlReader::getAnglePlugin()
{
	std::string anglePlugin;
	if ( executionPointer_->ConsumerPlugin().AngleDevice().present() )
		anglePlugin = executionPointer_->ConsumerPlugin().AngleDevice().get();
	else
	{
		cout << "Cannot get AngleDevice parameters, AngleDevice option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return anglePlugin;
}

std::string ExecutionXmlReader::getEmgPlugin()
{
	std::string emgPlugin;
	
	if ( executionPointer_->ConsumerPlugin().EMGDevice().present() )
		emgPlugin = executionPointer_->ConsumerPlugin().EMGDevice().get();
	else
	{
		cout << "Cannot get EMGDevice parameters, EMGDevice option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return emgPlugin;
}


std::string ExecutionXmlReader::getAngleFile()
{
	std::string AngleFile;
	if ( executionPointer_->ConsumerPlugin().AngleDeviceFile().present() )
		AngleFile = executionPointer_->ConsumerPlugin().AngleDeviceFile().get();
	else
	{
		cout << "Cannot get AngleDeviceFile parameters, AngleDeviceFile option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return AngleFile;
}


std::string ExecutionXmlReader::getEmgFile()
{
	std::string emgFile;
	if ( executionPointer_->ConsumerPlugin().EMGDeviceFile().present() )
		emgFile = executionPointer_->ConsumerPlugin().EMGDeviceFile().get();
	else
	{
		cout << "Cannot get EMGDeviceFile parameters, EMGDeviceFile option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return emgFile;
}

std::string ExecutionXmlReader::getComsumerPlugin()
{
	std::string comsumerPlugin;
	
	if ( executionPointer_->ConsumerPlugin().ComsumerDevice().present() )
		comsumerPlugin = executionPointer_->ConsumerPlugin().ComsumerDevice().get();
	else
	{
		cout << "Cannot get ComsumerFile parameters, ComsumerFile option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return comsumerPlugin;
}

std::string ExecutionXmlReader::getAngleAndComsumerPlugin()
{
	std::string angleAndComsumerPlugin;
	
	if ( executionPointer_->ConsumerPlugin().AngleAndComsumerDevice().present() )
		angleAndComsumerPlugin = executionPointer_->ConsumerPlugin().AngleAndComsumerDevice().get();
	else
	{
		cout << "Cannot get AngleAndComsumerDevice parameters, AngleAndComsumerDevice option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return angleAndComsumerPlugin;
}

std::string ExecutionXmlReader::getEmgAndAngleAndComsumerPlugin()
{
	std::string emgAndAngleAndComsumerPlugin;

	if (executionPointer_->ConsumerPlugin().EmgAndAngleAndComsumerDevice().present())
		emgAndAngleAndComsumerPlugin = executionPointer_->ConsumerPlugin().EmgAndAngleAndComsumerDevice().get();
	else
	{
		cout << "Cannot get EmgAndAngleAndComsumerDevice parameters, EmgAndAngleAndComsumerDevice option not selected\n" << std::flush;
		exit(EXIT_FAILURE);
	}

	return emgAndAngleAndComsumerPlugin;
}

std::string ExecutionXmlReader::getComsumerPort()
{
	std::string comsumerPort;
	
	if ( executionPointer_->ConsumerPlugin().ComsumerPort().present() )
		comsumerPort = executionPointer_->ConsumerPlugin().ComsumerPort().get();
	else
	{
		cout << "Cannot get ComsumerPort parameters, ComsumerPort option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	return comsumerPort;
}

std::string ExecutionXmlReader::getOptimizationPlugin()
{
	std::string OptimizationPlugin;

	if (executionPointer_->ConsumerPlugin().OptimizationDevice().present())
		OptimizationPlugin = executionPointer_->ConsumerPlugin().OptimizationDevice().get();
	else
	{
		cout << "Cannot get OptimizationDevice parameters, OptimizationDevice option not selected\n" << std::flush;
		exit(EXIT_FAILURE);
	}

	return OptimizationPlugin;
}

std::string ExecutionXmlReader::getEMGAndAnglePlugin()
{
	std::string angleAndComsumerPlugin;

	if (executionPointer_->ConsumerPlugin().EMGAndAngleDevice().present())
		angleAndComsumerPlugin = executionPointer_->ConsumerPlugin().EMGAndAngleDevice().get();
	else
	{
		cout << "Cannot get EMGAndAngleDevice parameters, EMGAndAngleDevice option not selected\n" << std::flush;
		exit(EXIT_FAILURE);
	}

	return angleAndComsumerPlugin;
}

std::string ExecutionXmlReader::getOptimizationFile()
{
	std::string OptimizationFile;
	if (executionPointer_->ConsumerPlugin().OptimizationFile().present())
		OptimizationFile = executionPointer_->ConsumerPlugin().OptimizationFile().get();
	else
	{
		cout << "Cannot get OptimizationFile parameters, OptimizationFile option not selected\n" << std::flush;
		exit(EXIT_FAILURE);
	}

	return OptimizationFile;
}

bool ExecutionXmlReader::useOfAnglePlugin()
{
	return executionPointer_->ConsumerPlugin().AngleDeviceFile().present();
}

bool ExecutionXmlReader::useOfEmgPlugin()
{
	return executionPointer_->ConsumerPlugin().EMGDevice().present();
}

bool ExecutionXmlReader::useOfComsumerPlugin()
{
	return executionPointer_->ConsumerPlugin().ComsumerDevice().present();
}

bool ExecutionXmlReader::useOfAngleAndComsumerPlugin()
{
	return executionPointer_->ConsumerPlugin().AngleAndComsumerDevice().present();
}

bool ExecutionXmlReader::useOfEmgAndAngleAndComsumerPlugin()
{
	return executionPointer_->ConsumerPlugin().EmgAndAngleAndComsumerDevice().present();
}

bool ExecutionXmlReader::useOfEMGAndAnglePlugin()
{
	return executionPointer_->ConsumerPlugin().EMGAndAngleDevice().present();
}

bool ExecutionXmlReader::useOfOptimizationPlugin()
{
	return executionPointer_->ConsumerPlugin().OptimizationDevice().present();
}

void ExecutionXmlReader::getMusclesToPredict ( std::vector<std::string>& musclesToPredict )
{

	ExecutionType::NMSmodel_type& myModel ( executionPointer_->NMSmodel() );
	ExecutionType::NMSmodel_type::type_type& myType ( myModel.type() );
	ExecutionType::NMSmodel_type::type_type::hybrid_optional& myHybOpt ( myType.hybrid() );

	if ( !myHybOpt.present() )
	{
		cout << "Cannot get hybrid parameters, hybrid option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}


	musclesToPredict.clear();
	HybridType::predictedMuscles_type& predictedMuscles ( myType.hybrid()->predictedMuscles() );

	for ( unsigned mi = 0; mi < predictedMuscles.size(); ++mi )
		musclesToPredict.push_back ( predictedMuscles.at ( mi ) );
}


void ExecutionXmlReader::getMusclesToTrack ( std::vector<std::string>& musclesToTrack )
{

	ExecutionType::NMSmodel_type& myModel ( executionPointer_->NMSmodel() );
	ExecutionType::NMSmodel_type::type_type& myType ( myModel.type() );
	ExecutionType::NMSmodel_type::type_type::hybrid_optional& myHybOpt ( myType.hybrid() );

	if ( !myHybOpt.present() )
	{
		cout << "Cannot get hybrid parameters, hybrid option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}


	musclesToTrack.clear();
	HybridType::trackedMuscles_type& trackedMuscles ( myType.hybrid()->trackedMuscles() );

	for ( unsigned mi = 0; mi < trackedMuscles.size(); ++mi )
		musclesToTrack.push_back ( trackedMuscles.at ( mi ) );
}

void ExecutionXmlReader::getHybridWeightings ( double& alpha, double& beta, double& gamma )
{

	ExecutionType::NMSmodel_type& myModel ( executionPointer_->NMSmodel() );
	ExecutionType::NMSmodel_type::type_type& myType ( myModel.type() );
	ExecutionType::NMSmodel_type::type_type::hybrid_optional& myHybOpt ( myType.hybrid() );

	if ( !myHybOpt.present() )
	{
		cout << "Cannot get hybrid parameters, hybrid option not selected\n" << std::flush;
		exit ( EXIT_FAILURE );
	}

	HybridType::alpha_type& myAlpha ( myType.hybrid()->alpha() );
	alpha = myAlpha;

	HybridType::beta_type& myBeta ( myType.hybrid()->beta() );
	beta = myBeta;

	HybridType::gamma_type& myGamma ( myType.hybrid()->gamma() );
	gamma = myGamma;
}

// void ExecutionXmlReader::getDynLib ( string& EMGDynLib, string& angleDynLib ) const
// {
// 	if ( isRealTime_ )
// 	{
// 		DevicePluginType::EMGDevice_type& EMGDevice ( executionPointer_->ConsumerPlugin().EMGDevice() );
// 		DevicePluginType::AngleDevice_type& angleDevice ( executionPointer_->ConsumerPlugin().AngleDevice() );
// 		EMGDynLib = string ( EMGDevice );
// 		angleDynLib = string ( angleDevice );
// 	}
// 	else
// 	{
// 		cout << "Error: Real Time option not chose." << endl;
// 		exit ( EXIT_FAILURE );
// 	}
// }

bool ExecutionXmlReader::isRealTime()
{
	return isRealTime_;
}

NMSModelCfg::RunMode ExecutionXmlReader::getRunMode() const
{

	return static_cast<NMSModelCfg::RunMode> ( runMode_ );

}
