#ifndef OPTIMISATIONPLUGIN_H_
#define OPTIMISATIONPLUGIN_H_

#include "NMSmodel.h"
#include "Activation/ExponentialActivation.h"
#include "Activation/ExponentialActivationRT.h"
#include "Tendon/StiffTendon.h"
#include "Tendon/ElasticTendon.h"
#include "Tendon/ElasticTendon_BiSec.h"

template <typename NMSmodelT>
class OptimizationPlugin
{
public:
	OptimizationPlugin() {};
	~OptimizationPlugin() {};
	virtual void init(NMSmodelT& model, const std::string& executionXMLFileName, const std::string& configurationFileName) = 0;
	virtual std::vector<double> getDofTorque() = 0;

	virtual std::vector<double> getShapeFactor() = 0;
	virtual std::vector<double> getTendonSlackLengths() = 0;
	virtual std::vector<double> getOptimalFiberLengths() = 0;
	virtual std::vector<double> getGroupMusclesBasedOnStrengthCoefficients() = 0;

	virtual void setMuscleForce(const std::vector<double>&) = 0;
	virtual void setMuscleActiveForce(const std::vector<double>&) = 0;
	virtual void setDOFTorque(const std::vector<double>&) = 0;
	virtual void setExternalDOFTorque(const std::vector<double>&) = 0;
	virtual void setActivations(const std::vector<double>&) = 0;
	virtual void setFibreLengths(const std::vector<double>&) = 0;
	virtual void setFibreVelocities(const std::vector<double>&) = 0;
	virtual void setPennationAngle(const std::vector<double>&) = 0;
	virtual void setTendonLength(const std::vector<double>&) = 0;
	virtual void setTime(const double&) = 0;
	virtual void setEmgs(const std::vector<double>&) = 0;
	virtual void setLmt(const std::vector<double>&) = 0;
	virtual void setMA(const std::vector<std::vector<double> >&) = 0;
	virtual void setJointAngle(const std::vector<double>& jointAngle) = 0;

	virtual void start() = 0;
	virtual void stop() = 0;
	virtual void setDirectories(const std::string& outDirectory, const std::string& inDirectory = std::string()) = 0;
	virtual void setVerbose(const int&) = 0;
	virtual void setRecord(const bool&) = 0;
	virtual void newData() = 0;
};


typedef OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* create_cEAS();
typedef void destroy_cEAS(OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >*);

typedef OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> >* create_cEASoff();
typedef void destroy_cEASoff(OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline> >*);

typedef OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >* create_cEAEB();
typedef void destroy_cEAEB(OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >*);

typedef OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* create_cEAE();
typedef void destroy_cEAE(OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >*);


#endif