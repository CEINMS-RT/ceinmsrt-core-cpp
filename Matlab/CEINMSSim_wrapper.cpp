

/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <ModelInterface.h>

namespace CEINMSSim
{

ModelInterface<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* modelInter;
std::vector<std::string> DOFName;
std::vector<std::string> MuscleName;
}

using namespace CEINMSSim;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 32
#define y_width 1
/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output functions
 *
 */
 
 void CEINMSSim_start_wrapper(const int8_T  *subjectSpecificXml, const int_T  p_width0,
	const int8_T  *ExecutionXML, const int_T  p_width1)
{
	char* subjectSpecificXmlChar = (char*)subjectSpecificXml;
	char* ExecutionXMLChar = (char*)ExecutionXML;
	
	std::string subjectSpecificXmlString = "cfg/TestData/gait2392Left.xml";
	std::string ExecutionXMLString = "cfg/TestData/executionRT_ankle_knee.xml";
	
	//std::string subjectSpecificXmlString(subjectSpecificXmlChar);
	//std::string ExecutionXMLString(ExecutionXMLChar);
	
	modelInter = new ModelInterface<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >(subjectSpecificXmlString, ExecutionXMLString);

	modelInter->initialize();

	DOFName = modelInter->getDOFName();
	MuscleName = modelInter->getMuscleName();

}
 
void CEINMSSim_Outputs_wrapper(const real_T *EMG,
			const real_T *LMT,
			const real_T *MA,
			const real_T *Time,
			real_T *Torque,
			real_T *MusclesForce,
			real_T *Activations,
			real_T *FibreLengths,
			real_T *FibreVelocity,
			real_T *PenAngle,
			real_T *TendonLength,
			const int8_T  *subjectSpecificXml, const int_T  p_width0,
			const int8_T  *ExecutionXML, const int_T  p_width1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
	std::vector<double> lmt;
	std::vector<double> emg;
	std::vector<std::vector<double> > ma;

	std::vector<double> activations;
	std::vector<double> fiberLength;
	std::vector<double> fiberVelocity;
	std::vector<double> pennationAngle;
	std::vector<double> tendonLength;
	std::vector<double> torque;
	std::vector<double> muscleForce;


	for (int i = 0; i < DOFName.size(); i++)
	{
		lmt.push_back(LMT[i]);
	}

	for (int i = 0; i < MuscleName.size(); i++)
	{
		emg.push_back(EMG[i]);
	}

	for (int i = 0; i < DOFName.size(); i++)
	{
		std::vector<unsigned> muscleIndex;
		modelInter->getMusclesIndexOnDof(muscleIndex, i);
		std::vector<double> MaOneDOF;
		for (int j = 0; j < muscleIndex.size(); j++)
		{
			MaOneDOF.push_back(MA[i * 32 + j]);
		}
		ma.push_back(MaOneDOF);
	}

	modelInter->setEMG(emg, double(Time[0]));
	modelInter->setLMTAndMA(lmt, ma, double(Time[0]));
	torque = modelInter->getMuscleForce();
	muscleForce = modelInter->getDOFTorque();
	activations = modelInter->getActivations();
	fiberLength = modelInter->getFibreLengths();
	fiberVelocity = modelInter->getFibreVelocities();
	pennationAngle = modelInter->getPennationAngle();
	tendonLength = modelInter->getTendonLength();

	for (std::vector<double>::const_iterator it = torque.begin(); it != torque.end(); it++)
	{
		Torque[std::distance<std::vector<double>::const_iterator>(torque.begin(), it)] = *it;
	}

	for (std::vector<double>::const_iterator it = muscleForce.begin(); it != muscleForce.end(); it++)
	{
		MusclesForce[std::distance<std::vector<double>::const_iterator>(muscleForce.begin(), it)] = *it;
	}

	for (std::vector<double>::const_iterator it = activations.begin(); it != activations.end(); it++)
	{
		Activations[std::distance<std::vector<double>::const_iterator>(activations.begin(), it)] = *it;
	}

	for (std::vector<double>::const_iterator it = fiberLength.begin(); it != fiberLength.end(); it++)
	{
		FibreLengths[std::distance<std::vector<double>::const_iterator>(fiberLength.begin(), it)] = *it;
	}

	for (std::vector<double>::const_iterator it = fiberVelocity.begin(); it != fiberVelocity.end(); it++)
	{
		FibreVelocity[std::distance<std::vector<double>::const_iterator>(fiberVelocity.begin(), it)] = *it;
	}

	for (std::vector<double>::const_iterator it = pennationAngle.begin(); it != pennationAngle.end(); it++)
	{
		PenAngle[std::distance<std::vector<double>::const_iterator>(pennationAngle.begin(), it)] = *it;
	}

	for (std::vector<double>::const_iterator it = tendonLength.begin(); it != tendonLength.end(); it++)
	{
		TendonLength[std::distance<std::vector<double>::const_iterator>(tendonLength.begin(), it)] = *it;
	}
	
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

void CEINMSSim_end_wrapper()
{
	delete modelInter;
}

