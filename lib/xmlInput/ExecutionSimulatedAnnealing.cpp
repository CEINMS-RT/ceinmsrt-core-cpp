// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

#include "ExecutionSimulatedAnnealing.h"

ExecutionSimulatedAnnealing::ExecutionSimulatedAnnealing(const std::string& simulatedAnnealingFile)
{
	//TODO add calibration option
	try
	{
		_annealingPointer = std::auto_ptr<SimulatedAnnealingType>(simulatedAnnealing(simulatedAnnealingFile,
			xml_schema::flags::dont_initialize));
	}
	catch (const xml_schema::exception& e)
	{
		std::cout << e << std::endl;
		exit(EXIT_FAILURE);
	}

	// 	const DofCalSequence& dofSeq = _annealingPointer->dofToCalibrate();

	// 	for ( DofCalSequence::const_iterator it = dofSeq.begin();
	// 			it != dofSeq.end(); it++ )
	// 		_DOFToCalibrate.push_back ( std::string ( *it ) );

	const Trials::trial_sequence& trialsSeq = _annealingPointer->calibration().trials().trial();

	for (Trials::trial_sequence::const_iterator it = trialsSeq.begin();
		it != trialsSeq.end(); it++)
	{
		_trialsName.push_back(it->trialName());
		_trialscropMin.push_back(it->cropMinTimeFromZero());
		_trialscropMax.push_back(it->cropMaxTimeFromZero());
	}

	const Calibration::dofToCalibrate_type::dofs_sequence& dofSeqCal = _annealingPointer->calibration().dofToCalibrate().dofs();

	std::set<std::string> dofToCalibrateSet;

	for (Calibration::dofToCalibrate_type::dofs_sequence::const_iterator it = dofSeqCal.begin();
		it != dofSeqCal.end(); it++)
	{
		const DofCalSequence& dofSeq = *it;
		std::vector<std::string> temp;

		for (DofCalSequence::const_iterator it = dofSeq.begin(); it != dofSeq.end(); it++)
		{
			temp.push_back(std::string(*it));
			dofToCalibrateSet.insert(std::string(*it));
		}

		_DOFSequenceToCalibrate.push_back(temp);
	}

	_DOFToCalibrate = std::vector<std::string>(dofToCalibrateSet.begin(), dofToCalibrateSet.end());

	if (_annealingPointer->EMGProccesing().present())
	{
		_EMGProccesing = _annealingPointer->EMGProccesing().get();
	}
	else
	{
		_EMGProccesing = "noFile";
		_filterEMG = false;
	}

	_useSpline = _annealingPointer->computeSpline().use();
	_scaledOsimFile = _annealingPointer->osimFile();

	if ( _annealingPointer->computeSpline().printingOption().present() )
		_printOption = _annealingPointer->computeSpline().printingOption().get();
	else
		_printOption = 0;

	if ( _annealingPointer->computeSpline().numberOfNode().present() )
		_numberOfNode = _annealingPointer->computeSpline().numberOfNode().get();
	else
		_numberOfNode = 9;

	_nameOfSubject = _annealingPointer->nameOfSubject();
	_subjectXML = _annealingPointer->subjectXML();
	_trialsDirectory = _annealingPointer->calibration().trials().directory();

	if (_annealingPointer->preScaling().present())
	{
		_usePreScaling = _annealingPointer->preScaling().get().use();
		_unscaledOsimFile = _annealingPointer->preScaling().get().unscaledOsimFile();
	}
	else
	{
		_usePreScaling = false;
	}
	
	_useCalibration = _annealingPointer->calibration().use();
	if(_annealingPointer->calibration().filterEMG().present())
		_filterEMG = _annealingPointer->calibration().filterEMG().get();
	else
		_filterEMG = false;
	
	_noEpsilon = _annealingPointer->calibration().option().noEpsilon();

	if ( _annealingPointer->calibration().option().NT().present () )
		nt_ = _annealingPointer->calibration().option().NT().get();
	else
		nt_ = 5;

	if ( _annealingPointer->calibration().option().NS().present () )
		ns_ = _annealingPointer->calibration().option().NS().get();
	else
		ns_ = 15;

	if ( _annealingPointer->calibration().option().RT().present () )
		rt_ = _annealingPointer->calibration().option().RT().get();
	else
		rt_ = 0.3;

	if ( _annealingPointer->calibration().option().T().present () )
		t_ = _annealingPointer->calibration().option().T().get();
	else
		t_ = 20;

	_epsilon = _annealingPointer->calibration().option().epsilon();

	maxNoEval_ = _annealingPointer->calibration().option().maxNoEval();

	inputSubjectXMLName_ = _annealingPointer->calibration().inputSubjectXMLName();
	outputSubjectXMLName_ = _annealingPointer->calibration().outputSubjectXMLName();

	emd_ = _annealingPointer->calibration().EMD().present() ? -_annealingPointer->calibration().EMD().get() : 0;
	
	if(_annealingPointer->calibration().objectiveFunction() == "ShapeFactor")
		calibMode_ = ShapeFactor;
	else if(_annealingPointer->calibration().objectiveFunction() == "StrengthCoefficients_ShapeFactor_TendonSlackLength_single")
		calibMode_ = StrengthCoefficients_ShapeFactor_TendonSlackLength_single;
	else if(_annealingPointer->calibration().objectiveFunction() == "GroupedStrengthCoefficients_IndividualLstLom")
		calibMode_ = GroupedStrengthCoefficients_IndividualLstLom;
	else if (_annealingPointer->calibration().objectiveFunction() == "StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange")
		calibMode_ = StrengthCoefficients_ShapeFactor_TendonSlackLength_single_widerRange;
	else
	{
		COUT << "Objective function unknow, we have:\nShapeFactor\nStrengthCoefficients_ShapeFactor_TendonSlackLength_single\nGroupedStrengthCoefficients_IndividualLstLom" <<  std::endl;
		exit(0);
	}
}

ExecutionSimulatedAnnealing::~ExecutionSimulatedAnnealing()
{

}
