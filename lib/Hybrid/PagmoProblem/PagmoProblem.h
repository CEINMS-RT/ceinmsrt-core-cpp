#ifndef PagmoProblem_h
#define PagmoProblem_h

#include "../HybridWeightings.h"
#include "../StaticComputation.h"
#include "../StaticComputationMode/Default.h"
#include "../Parameters/RecursiveEMGs.h"
#include <utility> // for std::pair
#include <vector>
#include <string>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <future>

#include "SetupDataStructure.h"

using namespace Hybrid;

template<typename NMSmodelT>
class PagmoProblem
{
public:
	PagmoProblem();
	/*PagmoProblem(const PagmoProblem& orig) : 
		configurationFile_(orig.configurationFile_),
		subjectDofNames_ (orig.subjectDofNames_),
		bounds_ (orig.bounds_),
		hybridParameters_ (orig.hybridParameters_),
		performanceCriterion_ (orig.performanceCriterion_),
		musclesNamesWithEmgToTrack_ (orig.musclesNamesWithEmgToTrack_),
		musclesNamesWithEmgToPredict_ (orig.musclesNamesWithEmgToPredict_),
		muscleIndexWithEMGtoTrack_ (orig.muscleIndexWithEMGtoTrack_),
		muscleIndexWithEMGtoPredict_ (orig.muscleIndexWithEMGtoPredict_),
		muscleIndexWithEMGtoOptimize_ (orig.muscleIndexWithEMGtoOptimize_),
		noParameters_ (orig.noParameters_),
		externalTorques_ (orig.externalTorques_),
		staticComputation_ (nullptr)
	{
		std::vector<unsigned int> dofsUsedDummy;
		//std::vector<std::string> dofName;
		std::cout << "copy constructor called" << std::endl;
		subject_ = new NMSmodelT();
		//configurationFile_ = orig.configurationFile_;
		setupSubject(*subject_,configurationFile_);
		//std::cout << "subject setup" << std::endl;
		subject_->eraseUnusedDofs({ "ankle_angle_l" }, dofsUsedDummy);
		
		//std::cout << "DOFs erased" << std::endl;

		std::vector<double> ma;
		//unsigned whichDof;
		std::vector<double> emgs;
		std::vector<double> lmt;

		orig.subject_->getEmgs(emgs);
		orig.subject_->getMuscleTendonLengths(lmt);
		subject_->setEmgs(emgs);
		subject_->setMuscleTendonLengths(lmt);

		for (int i = 0; i < subjectDofNames_.size(); i++) {
			orig.subject_->getMomentArmsOnDof(ma, i);
			subject_->setMomentArms(ma, i);
		}

		//std::cout << "orig.subject_:" << *orig.subject_ << std::endl;
		//std::cout << "before setting staticComputation_. subject_: " << *subject_ << std::endl;
		//std::cout << "done" << std::endl;

		staticComputation_ = new StaticComputation<NMSmodelT, StaticComputationMode::Default<NMSmodelT>>(*subject_, musclesNamesWithEmgToTrack_, musclesNamesWithEmgToPredict_);
		//std::cout << "staticComputation set" << std::endl;
	}*/
	
	~PagmoProblem();
	// Mandatory functions (see pagmo user-defined problem documentation):
	std::vector<double> fitness(const std::vector<double>& dv) const; // dv: decision vector (values of (optimized muscles) EMG)
	//std::vector<double> batch_fitness(const std::vector<double>& dvs) const;
	std::pair<std::vector<double>, std::vector<double>> get_bounds() const;

	// Set functions (init)
	//void setSubjectDofNames(std::vector<std::string> subjectDofNames) { subjectDofNames_ = subjectDofNames; }
	void setModel(NMSmodelT* subject); // NMSmodelT* subject
	//void setModel(const std::string configurationFile);
	//void setModel(const std::string configurationFile, unsigned numberOfThreads);
	void setWeightings(HybridWeightings hybridParameters) { hybridParameters_ = hybridParameters; }
	void setPerformanceCriterion(const std::string performanceCriterion) { performanceCriterion_ = performanceCriterion; }
	void setMusclesNamesWithEmgToTrack(const std::vector<std::string>& musclesNamesWithEmgToTrack);
	void setMusclesNamesWithEmgToPredict(const std::vector<std::string>& musclesNamesWithEmgToPredict);
	void setNoParameters(unsigned noParameters) { noParameters_ = noParameters; }
	void setParameters();
	void set_bounds(std::vector<double> lowerBounds, std::vector<double> upperBounds);
	void setMutex(boost::mutex& Mutex);

	// Set functions (loop)
	void setSingleExternalTorque(double externalTorque, const std::string& whichDof);
	void setStaticComputation();

	/*void setEMGs(const std::vector<double>& EMGs) { subject_->setEmgs(EMGs); }
	void setTime(const double& time) {subject_->setTime(time); }
	void setMuscleTendonLengths(const std::vector<double>& Lmt) { subject_->setMuscleTendonLengths(Lmt); }
	void setMomentArms(const std::vector<double>& Ma, unsigned whichDof) { subject_->setMomentArms(Ma, whichDof); }
	void getPastEmgs(std::vector<double>& pastEmgs) { subject_->getPastEmgs(pastEmgs); }*/
	/*void setEMGs(const std::vector<double>& EMGs) { for (int i = 0; i < numberOfThreads_; i++) { subject_.at(i)->setEmgs(EMGs); } }
	void setTime(const double& time) { for (int i = 0; i < numberOfThreads_; i++) { subject_.at(i)->setTime(time); } }
	void setMuscleTendonLengths(const std::vector<double>& Lmt) { for (int i = 0; i < numberOfThreads_; i++) { subject_.at(i)->setMuscleTendonLengths(Lmt); } }
	void setMomentArms(const std::vector<double>& Ma, unsigned whichDof) { for (int i = 0; i < numberOfThreads_; i++) { subject_.at(i)->setMomentArms(Ma, whichDof); } }
	void getPastEmgs(std::vector<double>& pastEmgs) { subject_.at(0)->getPastEmgs(pastEmgs); }*/
	
	// Objective function evaluation
	//void evalfp(int iIndividual, int iThread) const;
	double evalfp() const;

	// Output
	friend std::ostream& operator<< <> (std::ostream& output, const PagmoProblem<NMSmodelT>& pagmoProblem);

private:
	void setupSubject(NMSmodelT& mySubject, string configurationFile);

	//std::vector<NMSmodelT*> subject_;
	NMSmodelT* subject_; // can't be a reference, because a reference must be initialized
	//std::vector<StaticComputation<NMSmodelT, StaticComputationMode::Default<NMSmodelT> >*> staticComputation_;
	StaticComputation<NMSmodelT, StaticComputationMode::Default<NMSmodelT> >* staticComputation_{nullptr};
	//Parameters::RecursiveEMGs<NMSmodelT>* parameters_;
	//unsigned numberOfThreads_;
	//mutable std::vector<double> fp_;

	std::vector<std::string> subjectDofNames_;
	std::pair<std::vector<double>, std::vector<double>> bounds_;
	HybridWeightings hybridParameters_;
	std::string performanceCriterion_;
	//std::string configurationFile_;

	std::vector<std::string> musclesNamesWithEmgToTrack_;
	std::vector<std::string> musclesNamesWithEmgToPredict_;
	std::vector<unsigned> muscleIndexWithEMGtoTrack_;
	std::vector<unsigned> muscleIndexWithEMGtoPredict_;
	std::vector<unsigned> muscleIndexWithEMGtoOptimize_;
	unsigned noParameters_;

	std::vector<double> externalTorques_;

	boost::mutex* Mutex_; // this is a pointer because one mutex is needed for all problem instances. When creating multiple islands, the problem is copied so if this was not a pointer, the mutex would be copied
};

#include "PagmoProblem.cpp"

#endif