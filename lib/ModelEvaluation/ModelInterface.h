#ifndef MODELINTERFACE_H_
#define MODELINTERFACE_H_

#ifdef WIN32
#include <windows.h>
#endif
#include "NMSmodel.h"
#include "Activation/ExponentialActivation.h"
#include "Activation/ExponentialActivationRT.h"
#include "Tendon/StiffTendon.h"
#include "Tendon/ElasticTendon.h"
#include "Tendon/ElasticTendon_BiSec.h"
#include "ExecutionXmlReader.h"
#include <typeinfo>
#include "SetupDataStructure.h"

template <typename NMSmodelT>
class ModelInterface
{
public:
	ModelInterface();
	ModelInterface(const std::string& subjectSpecificXml, const std::string& ExecutionXML);
	~ModelInterface();

	int initialize(); //To be call otherwise crash

	void setEMG(std::vector<double> emg, double time);
	void setLMTAndMA(std::vector<double> lmt, std::vector<std::vector<double> > ma, double time);

	std::vector<double> getMuscleForce();
	std::vector<double> getDOFTorque();
	std::vector<double> getActivations();
	std::vector<double> getFibreLengths();
	std::vector<double> getFibreVelocities();
	std::vector<double> getPennationAngle();
	std::vector<double> getTendonLength();
	std::vector<double> getMuscleTendonLengths();
	std::vector<std::string> getMuscleName();
	std::vector<std::string> getDOFName();
	void getMusclesIndexOnDof(std::vector<unsigned>& muscleIndex, unsigned whichDof) const;

private:
	NMSmodelT* _model;
	ExecutionXmlReader* _executionCfg;
	std::string _subjectSpecificXml;
};

#endif