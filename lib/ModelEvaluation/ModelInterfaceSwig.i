%module ModelInterfaceSwig
%{
#include "ModelInterface.h"
%}

%include "std_vector.i"
%include "std_string.i"

using namespace std;

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
};

%template(ModelInterfaceElastic) ModelInterface<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
%template(ModelInterfaceStiff) ModelInterface<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
%template(ModelInterfaceElasticBisec) ModelInterface<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >;

namespace std {
   %template(vectors) vector<string>;
   %template(vectord) vector<double>;
   %template(vectordd) vector<vector<double> >;
   %template(vectorss) vector<vector<string> >;
};