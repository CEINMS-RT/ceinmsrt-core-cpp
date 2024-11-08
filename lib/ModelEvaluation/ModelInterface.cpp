#include <ModelInterface.h>

template <typename NMSmodelT>
ModelInterface<NMSmodelT>::ModelInterface()
{

}

template <typename NMSmodelT>
ModelInterface<NMSmodelT>::ModelInterface(const std::string& subjectSpecificXml, const std::string& ExecutionXML) :
_subjectSpecificXml(subjectSpecificXml)
{
	xercesc::XMLPlatformUtils::Initialize();
	_executionCfg = new ExecutionXmlReader(ExecutionXML);
}

template <typename NMSmodelT>
ModelInterface<NMSmodelT>::~ModelInterface()
{
	delete _executionCfg;
	delete _model;
}

template <typename NMSmodelT>
int ModelInterface<NMSmodelT>::initialize()
{
	NMSModelCfg::RunMode runMode = _executionCfg->getRunMode();

	switch (runMode)
	{
	case NMSModelCfg::RealTimeOpenLoopExponentialActivationStiffTendonOnline:
	{
		typedef NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
		if (typeid(MyNMSmodel) != typeid(NMSmodelT))
		{
			std::cout << "Wrong NMS model type: " << typeid(MyNMSmodel).name() << " in XML and version " << typeid(NMSmodelT).name() << " used." << std::endl;
			return 1;
		}
		_model = new NMSmodelT();

		SetupDataStructure<NMSmodelT, Curve<CurveMode::Online> > setupData(_subjectSpecificXml);
		setupData.createCurves();
		setupData.createMuscles(*_model);
		setupData.createDoFs(*_model);
		setupData.createMusclesNamesOnChannel(*_model);
		break;
	}

	case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonBiSecOnline:
	{
		typedef NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
		if (typeid(MyNMSmodel) != typeid(NMSmodelT))
		{
			std::cout << "Wrong NMS model type: " << typeid(MyNMSmodel).name() << " in XML and version " << typeid(NMSmodelT).name() << " used." << std::endl;
			return 1;
		}
		_model = new NMSmodelT();

		SetupDataStructure<NMSmodelT, Curve<CurveMode::Online> > setupData(_subjectSpecificXml);
		setupData.createCurves();
		setupData.createMuscles(*_model);
		setupData.createDoFs(*_model);
		setupData.createMusclesNamesOnChannel(*_model);
		break;
	}

	case NMSModelCfg::RealTimeOpenLoopExponentialActivationElasticTendonOnline:
	{
		typedef NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> MyNMSmodel;
		if (typeid(MyNMSmodel) != typeid(NMSmodelT))
		{
			std::cout << "Wrong NMS model type: " << typeid(MyNMSmodel).name() << " in XML and version " << typeid(NMSmodelT).name() << " used." << std::endl;
			return 1;
		}
		_model = new NMSmodelT();

		SetupDataStructure<NMSmodelT, Curve<CurveMode::Online> > setupData(_subjectSpecificXml);
		setupData.createCurves();
		setupData.createMuscles(*_model);
		setupData.createDoFs(*_model);
		setupData.createMusclesNamesOnChannel(*_model);
		break;
	}

	default:
		COUT << runMode << endl;
		COUT << "Implementation not available yet. Verify you XML configuration file\n";
		break;
	}
	return 0;
}

template <typename NMSmodelT>
void ModelInterface<NMSmodelT>::setEMG(std::vector<double> emg, double time)
{
	_model->setTime(time); // Update model time
	_model->setEmgs(emg); // Update EMG for the model
	_model->updateActivations(); // conpute activation
	_model->pushState();
}

template <typename NMSmodelT>
void ModelInterface<NMSmodelT>::setLMTAndMA(std::vector<double> lmt, std::vector<std::vector<double> > ma, double time)
{
	_model->setTime(time); // Update time for the model
	_model->setMuscleTendonLengths(lmt); // Update LMt for the model
	for (std::vector<std::vector<double> >::const_iterator it = ma.begin(); it != ma.end(); it++)
		_model->setMomentArms(*it, std::distance<std::vector<std::vector<double> >::const_iterator>(ma.begin(), it)); // update MA for the model
	_model->updateState(); // Compute the torque 
	_model->pushState();
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getMuscleForce()
{
	std::vector<double> muscleForce;
	_model->getMuscleForces(muscleForce); // get muscle force
	return muscleForce;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getDOFTorque()
{
	std::vector<double> dataTorque;
	_model->getTorques(dataTorque); // get the torque 
	return dataTorque;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getActivations()
{
	std::vector<double> data;
	_model->getActivations(data);
	return data;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getFibreLengths()
{
	std::vector<double> data;
	_model->getFiberLengths(data);
	return data;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getFibreVelocities()
{
	std::vector<double> data;
	_model->getFiberVelocities(data);
	return data;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getPennationAngle()
{
	std::vector<double> data;
	_model->getPennationAngleInst(data);
	return data;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getTendonLength()
{
	std::vector<double> data;
	_model->getTendonLength(data);
	return data;
}

template <typename NMSmodelT>
std::vector<double> ModelInterface<NMSmodelT>::getMuscleTendonLengths()
{
	std::vector<double> data;
	_model->getMuscleTendonLengths(data);
	return data;
}

template <typename NMSmodelT>
std::vector<std::string> ModelInterface<NMSmodelT>::getMuscleName()
{
	std::vector<std::string> MuscleName;
	_model->getMuscleNames(MuscleName);
	return MuscleName;
}

template <typename NMSmodelT>
std::vector<std::string> ModelInterface<NMSmodelT>::getDOFName()
{
	std::vector<std::string> DOFName;
	_model->getDoFNames(DOFName);
	return DOFName;
}

template <typename NMSmodelT>
void ModelInterface<NMSmodelT>::getMusclesIndexOnDof(std::vector<unsigned>& muscleIndex, unsigned whichDof) const
{
	_model->getMusclesIndexOnDof(muscleIndex, whichDof);
}

template class ModelInterface<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
template class ModelInterface<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >;
template class ModelInterface<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
