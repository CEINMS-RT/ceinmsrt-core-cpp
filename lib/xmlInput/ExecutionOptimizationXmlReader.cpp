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

#include "ExecutionOptimizationXmlReader.h"


ExecutionOptimizationXmlReader::ExecutionOptimizationXmlReader(const std::string& filename) : useOfMuscleInTheLoop_(false), simulatedAnealing_(false), lm_(false), simplex_(false)
{
	try
	{
		std::auto_ptr<OptimizationType> executionPointer(optimization(filename, xml_schema::flags::dont_initialize));
		executionPointer_ = executionPointer;
	}
	catch (const xml_schema::exception& e)
	{
		std::cout << e << std::endl << std::flush;
		exit(EXIT_FAILURE);
	}
	readXml();
}


ExecutionOptimizationXmlReader::~ExecutionOptimizationXmlReader()
{
}

bool ExecutionOptimizationXmlReader::UseOfMuscleInTheLoop() {
	return useOfMuscleInTheLoop_;
}

bool ExecutionOptimizationXmlReader::UseOfOnlineCalibration()
{
	return useOfOnlineCalibration_;
}

bool ExecutionOptimizationXmlReader::UseOfMuscleParameter()
{
	return useOfMuscleParameter_;
}

bool ExecutionOptimizationXmlReader::UseSimulatedAnnealing()
{
	return simulatedAnealing_;
}

bool ExecutionOptimizationXmlReader::UseSimplex()
{
	return simplex_;
}

bool ExecutionOptimizationXmlReader::UseLM()
{
	return lm_;
}


/*
	>> Functions to get values from external classes
*/


// Muscle In The Loop Optimization
std::vector<std::string> ExecutionOptimizationXmlReader::getMusclesList() {
	return musclesList_;
}

std::vector<std::string> ExecutionOptimizationXmlReader::getMusclesToOptimize() {
	return musclesToOptimize_;
}

std::vector<std::string> ExecutionOptimizationXmlReader::getDofsList() {
	return dofsList_;
}

std::string ExecutionOptimizationXmlReader::getOptimizationCriterion() {
	return optimizationCriterion_;
}

double ExecutionOptimizationXmlReader::getWheightOptimizedMuscles() {
	return wheightOptimizedMuscles_;
}

double ExecutionOptimizationXmlReader::getWheightNonOptimizedMuscles() {
	return wheightNonOptimizedMuscles_;
}

double ExecutionOptimizationXmlReader::getReductionFactor() {
	return reductionFactor_;
}

int ExecutionOptimizationXmlReader::getnCyclesRef() {
	return nCyclesRef_;
}

int ExecutionOptimizationXmlReader::getnCyclesOptimization() {
	return nCyclesOptimization_;
}


// Simulated Annealing
double ExecutionOptimizationXmlReader::getNoEpsilon()
{
	return noEpsilon_;
}

double ExecutionOptimizationXmlReader::getRt()
{
	return rt_;
}

double ExecutionOptimizationXmlReader::getT()
{
	return t_;
}

int ExecutionOptimizationXmlReader::getNS()
{
	return ns_;
}

int ExecutionOptimizationXmlReader::getNT()
{
	return nt_;
}

double ExecutionOptimizationXmlReader::getEpsilon()
{
	return epsilon_;
}

int ExecutionOptimizationXmlReader::getMaxNoEval()
{
	return maxNoEval_;
}

int ExecutionOptimizationXmlReader::getBufferSize()
{
	return bufferSize_;
}

// >> Read the XML file
void ExecutionOptimizationXmlReader::readXml() {

	useOfMuscleInTheLoop_ = executionPointer_->MuscleInTheLoop().present(); // To see if it is present in the document
	// Using the variables as in the definition in executionOptimization.hxx
	if (useOfMuscleInTheLoop_) {

		OptimizationType::MuscleInTheLoop_type::trackedMuscles_type& trackedMuscles = executionPointer_->MuscleInTheLoop().get().trackedMuscles();
		for (OptimizationType::MuscleInTheLoop_type::trackedMuscles_type::const_iterator it = trackedMuscles.begin(); it != trackedMuscles.end(); it++)
			musclesList_.push_back(*it);

		OptimizationType::MuscleInTheLoop_type::optimizedMuscles_type& optimizedMuscles = executionPointer_->MuscleInTheLoop().get().optimizedMuscles();
		for (OptimizationType::MuscleInTheLoop_type::optimizedMuscles_type::const_iterator it = optimizedMuscles.begin(); it != optimizedMuscles.end(); it++)
			musclesToOptimize_.push_back(*it);

		OptimizationType::MuscleInTheLoop_type::DOFsOptimized_type& DOFsOptimized = executionPointer_->MuscleInTheLoop().get().DOFsOptimized();
		for (OptimizationType::MuscleInTheLoop_type::DOFsOptimized_type::const_iterator it = DOFsOptimized.begin(); it != DOFsOptimized.end(); it++)
			dofsList_.push_back(*it);

		optimizationCriterion_ = executionPointer_->MuscleInTheLoop().get().optimizationCriterion();
		wheightOptimizedMuscles_ = executionPointer_->MuscleInTheLoop().get().wheightOptimizedMuscle();
		wheightNonOptimizedMuscles_ = executionPointer_->MuscleInTheLoop().get().wheightNonOptimizedMuscle();
		reductionFactor_ = executionPointer_->MuscleInTheLoop().get().wheightNonOptimizedMuscle();
		nCyclesRef_ = executionPointer_->MuscleInTheLoop().get().nCyclesRef();
		nCyclesOptimization_ = executionPointer_->MuscleInTheLoop().get().nCyclesOptimization();



	}


	useOfOnlineCalibration_ = executionPointer_->OnlineCalibration().present();
	if (useOfOnlineCalibration_)
	{
		bufferSize_ = executionPointer_->OnlineCalibration().get().BufferSize();
	}


	simulatedAnealing_ = executionPointer_->Algorithm().SimulatedAnnealing().present();
	if (simulatedAnealing_)
	{
		noEpsilon_ = executionPointer_->Algorithm().SimulatedAnnealing().get().noEpsilon();
		rt_ = executionPointer_->Algorithm().SimulatedAnnealing().get().rt();
		t_ = executionPointer_->Algorithm().SimulatedAnnealing().get().T();
		ns_ = executionPointer_->Algorithm().SimulatedAnnealing().get().NS();
		nt_ = executionPointer_->Algorithm().SimulatedAnnealing().get().NT();
		epsilon_ = executionPointer_->Algorithm().SimulatedAnnealing().get().epsilon();
		maxNoEval_ = executionPointer_->Algorithm().SimulatedAnnealing().get().maxNoEval();
	}
	else
	{
		nt_ = 5;
		ns_ = 20;
		rt_ = .4;
		t_ = 20;
		maxNoEval_ = 200000000;
		epsilon_ = 1e-4;
		noEpsilon_ = 8;
	}


	lm_ = executionPointer_->Algorithm().LM().present();
	simplex_ = executionPointer_->Algorithm().Simplex().present();
}