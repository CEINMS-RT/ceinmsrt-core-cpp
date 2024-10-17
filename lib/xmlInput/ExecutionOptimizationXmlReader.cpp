#include "ExecutionOptimizationXmlReader.h"


ExecutionOptimizationXmlReader::ExecutionOptimizationXmlReader(const std::string& filename) : useOfMuscleInTheLoop_(false), useOfHybrid_(false), simulatedAnealing_(false), lm_(false), simplex_(false)
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

bool ExecutionOptimizationXmlReader::UseOfHybrid()
{
	return useOfHybrid_;
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


// Hybrid Optimization
std::vector<std::string> ExecutionOptimizationXmlReader::getHybridMuscleWithEMGToPredict()
{
	return hybridMuscleWithEMGToPredict_;
}

std::vector<std::string> ExecutionOptimizationXmlReader::getHybridMuscleWithEMG()
{
	return hybridMuscleWithEMG_;
}

HybridWeightings ExecutionOptimizationXmlReader::getHybridWeightings()
{
	return hybridWeightings_;
}

std::string ExecutionOptimizationXmlReader::getPerformanceCriterion()
{
	return performanceCriterion_;
}

std::vector<std::string> ExecutionOptimizationXmlReader::getHybridDOFsOptimized()
{
	return hybridDOFsOptimized_;
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

std::vector<double> ExecutionOptimizationXmlReader::getMuscleForceTreshold()
{
	return muscleForceTreshold_;
}

std::vector<double> ExecutionOptimizationXmlReader::getMuscleLengthTreshold()
{
	return muscleLengthTreshold_;
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


	useOfHybrid_ = executionPointer_->Hybrid().present();
	if (useOfHybrid_)
	{
		OptimizationType::Hybrid_type::predictedMuscles_type& predictedMuscles = executionPointer_->Hybrid().get().predictedMuscles();
		OptimizationType::Hybrid_type::trackedMuscles_type& trackedMuscles = executionPointer_->Hybrid().get().trackedMuscles();

		for (OptimizationType::Hybrid_type::predictedMuscles_type::const_iterator it = predictedMuscles.begin(); it != predictedMuscles.end(); it++)
			hybridMuscleWithEMGToPredict_.push_back(*it);

		for (OptimizationType::Hybrid_type::trackedMuscles_type::const_iterator it = trackedMuscles.begin(); it != trackedMuscles.end(); it++)
			hybridMuscleWithEMG_.push_back(*it);

		hybridWeightings_.alpha = executionPointer_->Hybrid().get().alpha();
		hybridWeightings_.beta = executionPointer_->Hybrid().get().beta();
		hybridWeightings_.gamma = executionPointer_->Hybrid().get().gamma();

		OptimizationType::Hybrid_type::DOFsOptimized_type& DOFsOptimized = executionPointer_->Hybrid().get().DOFsOptimized();
		for (OptimizationType::Hybrid_type::DOFsOptimized_type::const_iterator it = DOFsOptimized.begin(); it != DOFsOptimized.end(); it++)
			hybridDOFsOptimized_.push_back(*it);

		performanceCriterion_ = executionPointer_->Hybrid().get().performanceCriterion();
	}


	useOfOnlineCalibration_ = executionPointer_->OnlineCalibration().present();
	if (useOfOnlineCalibration_)
	{
		OptimizationType::Hybrid_type::predictedMuscles_type& predictedMuscles = executionPointer_->OnlineCalibration().get().predictedMuscles();
		OptimizationType::Hybrid_type::trackedMuscles_type& trackedMuscles = executionPointer_->OnlineCalibration().get().trackedMuscles();

		for (OptimizationType::Hybrid_type::predictedMuscles_type::const_iterator it = predictedMuscles.begin(); it != predictedMuscles.end(); it++)
			hybridMuscleWithEMGToPredict_.push_back(*it);

		for (OptimizationType::Hybrid_type::trackedMuscles_type::const_iterator it = trackedMuscles.begin(); it != trackedMuscles.end(); it++)
			hybridMuscleWithEMG_.push_back(*it);

		bufferSize_ = executionPointer_->OnlineCalibration().get().BufferSize();
	}


	useOfMuscleParameter_ = executionPointer_->MuscleParameter().present();
	if (useOfMuscleParameter_)
	{
		OptimizationType::Hybrid_type::predictedMuscles_type& predictedMuscles = executionPointer_->MuscleParameter().get().trackedMuscles();

		for (OptimizationType::Hybrid_type::predictedMuscles_type::const_iterator it = predictedMuscles.begin(); it != predictedMuscles.end(); it++)
			hybridMuscleWithEMGToPredict_.push_back(*it);

		OptimizationType::MuscleParameter_type::MuscleForceTreshold_type& muscleForceTreshold = executionPointer_->MuscleParameter().get().MuscleForceTreshold();
		OptimizationType::MuscleParameter_type::MuscleForceTreshold_type& muscleLengthTreshold = executionPointer_->MuscleParameter().get().MuscleLengthTreshold();

		for (OptimizationType::MuscleParameter_type::MuscleForceTreshold_type::const_iterator it = muscleForceTreshold.begin(); it != muscleForceTreshold.end(); it++)
			muscleForceTreshold_.push_back(*it);

		for (OptimizationType::MuscleParameter_type::MuscleForceTreshold_type::const_iterator it = muscleLengthTreshold.begin(); it != muscleLengthTreshold.end(); it++)
			muscleLengthTreshold_.push_back(*it);

		hybridWeightings_.alpha = executionPointer_->MuscleParameter().get().alpha();
		hybridWeightings_.beta = executionPointer_->MuscleParameter().get().beta();
		hybridWeightings_.gamma = executionPointer_->MuscleParameter().get().gamma();
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