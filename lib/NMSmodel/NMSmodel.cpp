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

#include <iostream>
using std::cout;
using std::endl;
#include <vector>
using std::vector;
#include "NMSmodel.h"
#include <string>
using std::string;
#include "DoF.h"
#include "MuscleParameters.h"

#include <algorithm>

//#define DEBUG

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::addMuscle(const MTUtype& muscle) {
	muscles_.push_back(muscle);
	muscleNames_.push_back(muscle.getMuscleName());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::addDoF(const DoFtype& dof) {
	dofs_.push_back(dof);
	dofNames_.push_back(dof.getName());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::eraseUnusedDofs(const std::vector<std::string>& dofNamesUsed, std::vector<unsigned int>& dofsNotErased) {
	auto dofNameIt = dofNames_.begin();
	auto dofIt = dofs_.begin();
	dofsNotErased.clear();
	unsigned int i{ 0 };
	while (dofNameIt!= dofNames_.end())
	{
		if (std::find(dofNamesUsed.begin(), dofNamesUsed.end(), *dofNameIt) == dofNamesUsed.end()) // if the DOF is not in dofName, erase the DOF
		{
			dofIt = dofs_.erase(dofIt);
			dofNameIt = dofNames_.erase(dofNameIt);
		}
		else
		{
			++dofNameIt;
			++dofIt;
			dofsNotErased.push_back(i);					// now we know which DOFs are not erased, so we know which MAs to use
		}
		i++;
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
bool NMSmodel<Activation, Tendon, mode>::haveThisMuscle(const string& currentMuscle,
	vectorMTUitr& found) {
	found = muscles_.begin();
	while ((found != muscles_.end()) && (!found->compareMusclesId(currentMuscle)))
		found++;

	if (found == muscles_.end())
		return false;
	else
		return true;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
bool NMSmodel<Activation, Tendon, mode>::haveTheseMuscles(const vector<string>& musclesNames) {
	vectorMTUitr found;
	for (vector<string>::const_iterator mIt = musclesNames.begin(); mIt != musclesNames.end(); ++mIt)
		if (!haveThisMuscle(*mIt, found))
			return false;
	return true;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
bool NMSmodel<Activation, Tendon, mode>::compareMusclesNames(const vector<string>& muscleNames) const {
	vector<string>::const_iterator m1 = muscleNames.begin();
	for (vectorMTUconstItr m2 = muscles_.begin();
		m2 != muscles_.end();) {
		if (!(*m2).compareMusclesId(*m1))
			return false;
		m1++; m2++;
	}

	return true;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
bool NMSmodel<Activation, Tendon, mode>::compareMusclesNamesOnDoF(const vector<string>& muscleNames,
	int whichDoF) const {
	return dofs_.at(whichDoF).compareMusclesNames(muscleNames);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setTime(const double& time) {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
		muscleIt->setTime(time);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setEmgs(const std::vector<double>& currentEmgData) {
	vector<double>::const_iterator emgIt;
	vectorMTUitr muscleIt = muscles_.begin();
	for (emgIt = currentEmgData.begin(); emgIt < currentEmgData.end(); ++emgIt) {
		muscleIt->setEmg(*emgIt);
		++muscleIt;
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setEmgsSelective(const std::vector<double>& currentEmgData,
	const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<double>::const_iterator emgIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		emgIt = currentEmgData.begin() + *it;
		muscleIt->setEmg(*emgIt);
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setTime_emgs_updateActivations_pushState_selective(double time,
	const vector<double>& currentEmgData,
	const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<double>::const_iterator emgIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		emgIt = currentEmgData.begin() + *it;
		muscleIt->setTime(time);
		muscleIt->setEmg(*emgIt);
		muscleIt->updateActivation();
		muscleIt->pushState();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setMuscleTendonLengths(const vector<double>& currentLmtData) {
	vector<double>::const_iterator lmtIt;
	vectorMTUitr muscleIt = muscles_.begin();
	for (lmtIt = currentLmtData.begin(); lmtIt < currentLmtData.end(); ++lmtIt) {
		muscleIt->setMuscleTendonLength(*lmtIt);
		++muscleIt;
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setMuscleTendonLengthsSelective(const std::vector<double>& currentLmtData,
	const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<double>::const_iterator lmtIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		lmtIt = currentLmtData.begin() + *it;
		muscleIt->setMuscleTendonLength(*lmtIt);
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setMomentArms(const vector<double>& currentMaData, unsigned whichDof) {
	dofs_.at(whichDof).setMomentArms(currentMaData);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateState() {
	updateActivations();
	// 					{boost::timer::auto_cpu_timer auto_t;
	updateFibreLengthsAndVelocities();
	// 					}std::cout << std::flush;
	updateMuscleForces();
	updateTorques();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateState(const vector<unsigned>& selectedmusclesIndex) {
	updateActivations(selectedmusclesIndex);
	updateFibreLengthsAndVelocities(selectedmusclesIndex);
	updateMuscleForces(selectedmusclesIndex);
	updateTorques();
}

// template <>
// void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateState() {
//
//     updateActivations();
//     updateFibreLengthsAndVelocities();
//     updateMuscleForces();
//     updateTorques();
// }
//
//
// template <>
// void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateState(const vector<unsigned>& selectedmusclesIndex) {
//
//     updateActivations(selectedmusclesIndex);
//     updateFibreLengthsAndVelocities(selectedmusclesIndex);
//     updateMuscleForces(selectedmusclesIndex);
//     updateTorques();
// }
//

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateState_HYBRID() {
	updateActivations();
	updateFibreLengthsAndVelocities_HYBRID();
	updateMuscleForces();
	updateTorques();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateState_HYBRID(const vector<unsigned>& selectedmusclesIndex) {
	updateActivations(selectedmusclesIndex);
	updateFibreLengthsAndVelocities_HYBRID(selectedmusclesIndex);
	updateMuscleForces(selectedmusclesIndex);
	updateTorques();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateActivations() {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
		muscleIt->updateActivation();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateActivations(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->updateActivation();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengthsAndVelocities() {
	//  {boost::timer::auto_cpu_timer auto_t;
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
	{
		muscleIt->updateFibreLengthAndVelocity();
		// 		COUT << muscleIt->getMuscleName() << std::endl <<  std::flush;
	}
	// 	}std::cout << std::flush;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengthsAndVelocities(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->updateFibreLengthAndVelocity();
	}
}

// template <>
// void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateFibreLengthsAndVelocities() {
//
//     vectorMTUitr muscleIt = muscles_.begin();
//     for(muscleIt; muscleIt != muscles_.end(); ++muscleIt)
//         muscleIt->updateFibreLengthAndVelocity();
// }
//
//
// template <>
// void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateFibreLengthsAndVelocities(const vector<unsigned>& selectedMusclesIndex) {
//
//     vectorMTUitr muscleIt;
//     vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
//     for(it; it != selectedMusclesIndex.end(); ++it) {
//         muscleIt = muscles_.begin() + *it;
//         muscleIt->updateFibreLengthAndVelocity();
//     }
// }

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengthsAndVelocities_HYBRID() {
	vectorMTUitr muscleIt = muscles_.begin();
	//std::cout << "ok" << std::endl << std::flush;
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
		muscleIt->updateFibreLengthAndVelocity_HYBRID();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengthsAndVelocities_HYBRID(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->updateFibreLengthAndVelocity_HYBRID();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengths_OFFLINEPREP() {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
		muscleIt->updateFibreLength_OFFLINEPREP();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengths_OFFLINEPREP(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->updateFibreLength_OFFLINEPREP();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateMuscleForces() {
	//std::cout << "updateMuscleForces called" << std::endl;
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
		muscleIt->updateMuscleForce();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateMuscleForces(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->updateMuscleForce();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateTorques() {
	vectorDoFitr dofIt = dofs_.begin();
	for (dofIt; dofIt != dofs_.end(); ++dofIt)
		dofIt->updateTorque();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::pushState() {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt; muscleIt != muscles_.end(); ++muscleIt)
		muscleIt->pushState();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::pushState(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->pushState();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setMuscleForces(const std::vector<double>& muscleForces) {
	vector<double>::const_iterator mtfIt;
	vectorMTUitr muscleIt = muscles_.begin();
	for (mtfIt = muscleForces.begin(); mtfIt < muscleForces.end(); ++mtfIt) {
		muscleIt->setMuscleForce(*mtfIt);
		++muscleIt;
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setActivations(const std::vector<double>& activations) {
	vector<double>::const_iterator actIt;
	vectorMTUitr muscleIt = muscles_.begin();
	for (actIt = activations.begin(); actIt < activations.end(); ++actIt) {
		muscleIt->setActivation(*actIt);
		++muscleIt;
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengthTraces() {
	// usata
	vectorMTUitr muscleIt;
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		muscleIt->updateFibreLengthTrace();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::updateFibreLengthTraces(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->updateFibreLengthTrace();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getActivations(vector<double>& activations) const {
	activations.clear();
	activations.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		activations.push_back(muscleIt->getActivation());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getEmgs(vector<double>& currentEMGData) const {
	currentEMGData.clear();
	currentEMGData.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
		currentEMGData.push_back(muscleIt->getEmg());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getPastEmgs(vector<double>& pastEMGData) const {
	pastEMGData.clear();
	pastEMGData.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
		pastEMGData.push_back(muscleIt->getPastEmg());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getNeuralActivations(std::vector<double>& neuralActivations) const {
	neuralActivations.clear();
	neuralActivations.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
		neuralActivations.push_back(muscleIt->getNeuralActivation());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getFiberLengths(vector<double>& fiberLengths) const {
	fiberLengths.clear();
	fiberLengths.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		fiberLengths.push_back(muscleIt->getFiberLength());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getNormFiberLengths(std::vector<double>& normFiberLengths) const{
	normFiberLengths.clear();
	normFiberLengths.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		normFiberLengths.push_back(muscleIt->getNormFiberLength());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getFiberVelocities(vector<double>& fiberVelocities) const {
	fiberVelocities.clear();
	fiberVelocities.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		fiberVelocities.push_back(muscleIt->getFiberVelocity());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getNormFiberVelocities(vector<double>& normFiberVelocities) const {
	normFiberVelocities.clear();
	normFiberVelocities.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		normFiberVelocities.push_back(muscleIt->getNormFiberVelocity());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getNormFiberVelocitiesFilteredByMusclesList(std::vector<double>& normFiberVelocities, const std::vector<std::string>& musclesList) const
{
	normFiberVelocities.clear();
	normFiberVelocities.reserve(musclesList.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
	{
		for (std::vector<std::string>::const_iterator itString = musclesList.begin(); itString != musclesList.end(); itString++)
			if (muscleIt->compareMusclesId(*itString))
			{
				normFiberVelocities.push_back(muscleIt->getNormFiberVelocity());
				break;
			}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMuscleTendonLengthsFilteredByMusclesList(std::vector<double>& currentLmtData, const std::vector<std::string>& musclesList) const
{
	currentLmtData.clear();
	currentLmtData.reserve(musclesList.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
	{
		for (std::vector<std::string>::const_iterator itString = musclesList.begin(); itString != musclesList.end(); itString++)
			if (muscleIt->compareMusclesId(*itString))
			{
				currentLmtData.push_back(muscleIt->getMuscleTendonLength());
				break;
			}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMuscleForcesFilteredByMusclesList(std::vector<double>& muscleForces, const std::vector<std::string>& musclesList) const
{
	muscleForces.clear();
	muscleForces.reserve(musclesList.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
	{
		for (std::vector<std::string>::const_iterator itString = musclesList.begin(); itString != musclesList.end(); itString++)
			if (muscleIt->compareMusclesId(*itString))
			{
				muscleForces.push_back(muscleIt->getMuscleForce());
				break;
			}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMuscleForces(vector<double>& muscleForces) const {
	muscleForces.clear();
	muscleForces.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		muscleForces.push_back(muscleIt->getMuscleForce());
	}
}


template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclePassiveForces(std::vector<double>& muscleForcesPassive) const
{
	muscleForcesPassive.clear();
	muscleForcesPassive.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		muscleForcesPassive.push_back(muscleIt->getMusclePassiveForce());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMuscleActiveForces(std::vector<double>& muscleForcesActive) const
{
	muscleForcesActive.clear();
	muscleForcesActive.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		muscleForcesActive.push_back(muscleIt->getMuscleActiveForce());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getTorques(vector<double>& torques) const {
	torques.clear();
	torques.reserve(dofs_.size());
	vectorDoFconstItr dofIt = dofs_.begin();
	for (dofIt = dofs_.begin(); dofIt < dofs_.end(); ++dofIt)
	{
		torques.push_back(dofIt->getTorque());
		//std::cout << "gettorque: " << dofIt->getTorque() << std::endl;
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMuscleTendonLengths(std::vector<double>& currentLmtData) const
{
	currentLmtData.clear();
	currentLmtData.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		currentLmtData.push_back(muscleIt->getMuscleTendonLength());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getPennationAngle(std::vector<double>& pennationAngle) const
{
	pennationAngle.clear();
	pennationAngle.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		pennationAngle.push_back(muscleIt->getPennationAngle());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getPennationAngleInst(std::vector<double>& pennationAngle) const
{
	pennationAngle.clear();
	pennationAngle.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		pennationAngle.push_back(muscleIt->getPennationAngleInst());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getTendonLength(std::vector<double>& TendonLength) const
{
	TendonLength.clear();
	TendonLength.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		TendonLength.push_back(muscleIt->getTendonLength());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getTendonStrain(std::vector<double>& TendonStrain) const
{
	TendonStrain.clear();
	TendonStrain.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt != muscles_.end(); ++muscleIt)
	{
		TendonStrain.push_back(muscleIt->getTendonStrain());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getTendonSlackLengths(vector<double>& tendonSlackLengths) const {
	tendonSlackLengths.clear();
	tendonSlackLengths.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		tendonSlackLengths.push_back(muscleIt->getTendonSlackLength());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getOptimalFiberLengths(std::vector<double>& optimalFiberLengths) const
{
	optimalFiberLengths.clear();
	optimalFiberLengths.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		optimalFiberLengths.push_back(muscleIt->getOptimalFiberLength());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getShapeFactors(vector<double>& shapeFactors) const {
	shapeFactors.clear();
	shapeFactors.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		shapeFactors.push_back(muscleIt->getShapeFactor());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getC1Coefficients(vector<double>& c1Coefficients) const {
	c1Coefficients.clear();
	c1Coefficients.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		c1Coefficients.push_back(muscleIt->getC1());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getC2Coefficients(vector<double>& c2Coefficients) const {
	c2Coefficients.clear();
	c2Coefficients.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		c2Coefficients.push_back(muscleIt->getC2());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getstrengthCoefficientWithMaxIsometrics(vector<double>& isometricMuscleForce) const{
	isometricMuscleForce.clear();
	isometricMuscleForce.reserve(muscles_.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		isometricMuscleForce.push_back(muscleIt->getMaxIsometricForce() * muscleIt->getStrengthCoefficient());
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclesIndexOnDof(vector<unsigned>& musclesIndex, unsigned whichDof) const {
	musclesIndex.clear();
	vector<string> muscleNamesOnDof;
	dofs_.at(whichDof).getMusclesNamesOnDof(muscleNamesOnDof);
	vector<string>::const_iterator mnIt = muscleNamesOnDof.begin(), pos;
	for (mnIt; mnIt != muscleNamesOnDof.end(); ++mnIt) {
		//cout << *mnIt << endl;
		pos = std::find(muscleNames_.begin(), muscleNames_.end(), *mnIt);
		musclesIndex.push_back(std::distance(muscleNames_.begin(), pos));
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclesIndexFromLastDof(vector<unsigned int>& musclesIndexList,
	const vector<string>& whichDofs) {
	vector<string> lastDof;
	lastDof.push_back(whichDofs.back());
	getMusclesIndexFromDofs(musclesIndexList, lastDof);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclesIndexFromDofs(vector<unsigned>& musclesIndexList,
	const vector<string>& whichDofs) {
	musclesIndexList.clear();
	vector<string> currentMusclesList;
	vector<string> musclesList;
	bool found;

	for (unsigned i = 0; i < whichDofs.size(); ++i) {
		found = false;
		for (unsigned j = 0; j < dofs_.size(); ++j) {
			// 			cout << dofs_.at(j).getName();
			if (whichDofs.at(i) == dofs_.at(j).getName()) {
				dofs_.at(j).getMusclesNamesOnDof(currentMusclesList);
				for (unsigned k = 0; k < currentMusclesList.size(); ++k) {
					musclesList.push_back(currentMusclesList.at(k));
					//cout << currentMusclesList.at(k) << endl;
				}
				found = true;
			}
		}
		if (!found) {
			cout << whichDofs.at(i) << " not found on current subject, impossible to calibrate\n";
			cout << std::flush;
			exit(EXIT_FAILURE);
		}
	}

	//	for(vector<string>::const_iterator it = musclesList.begin(); it < musclesList.end(); it++)
	//	{
	//		cout << *it << endl;
	//	}
	//	cout << endl;

	std::sort(musclesList.begin(), musclesList.end()); //sort all muscles names in the vector
	std::unique(musclesList.begin(), musclesList.end()); //removes duplicated muscles

	//    for(vector<string>::const_iterator it = musclesList.begin(); it < musclesList.end(); it++)
	//    {
	//    	cout << *it << endl;
	//    }

	//    unsigned mlI = 0;
	//    for(unsigned i = 0; i < muscles_.size() && mlI < musclesList.size(); ++i) {
	//    	//cout << "musclesList.at(mlI): " << musclesList.at(mlI) << endl;
	//        if( muscles_.at(i).compareMusclesId(musclesList.at(mlI)) ) {
	//            musclesIndexList.push_back(i);
	//            ++mlI;
	//        }
	//    }

	typedef typename std::vector<MTUtype>::const_iterator MTUVectCI;
	for (MTUVectCI itMTU = muscles_.begin(); itMTU < muscles_.end(); itMTU++)
	{
		for (vector<string>::const_iterator it = musclesList.begin(); it < musclesList.end(); it++)
		{
			if (itMTU->compareMusclesId(*it))
			{
				musclesIndexList.push_back(std::distance<MTUVectCI>(muscles_.begin(), itMTU));
				break;
			}
		}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclesIndexFromMusclesList(std::vector<unsigned>& muscleIndexList, const vector<string>& musclesList) {
	//  cout << "you should test this function before using it\n";
	muscleIndexList.clear();
	muscleIndexList.reserve(musclesList.size());
	vectorMTUconstItr foundIt;
	unsigned j;

	for (unsigned i = 0; i < musclesList.size(); ++i) {
		foundIt = muscles_.begin(); j = 0;
		while ((foundIt != muscles_.end()) && (!foundIt->compareMusclesId(musclesList.at(i)))) {
			foundIt++; ++j;
		}
		if (foundIt != muscles_.end())
			muscleIndexList.push_back(j);
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getGroupMusclesBasedOnStrengthCoefficients(vector<double>& values,
	vector< vector<int> >& muscleGroups) const {
	values.clear();
	muscleGroups.clear();

	unsigned int currentMuscle = 0;
	for (currentMuscle = 0; currentMuscle < muscles_.size(); ++currentMuscle) {
		double currentStrengthCoefficient = muscles_.at(currentMuscle).getStrengthCoefficient();
		bool found = false;
		vector<double>::const_iterator vIt;
		vector< vector< int > >::iterator mgIt;
		for (vIt = values.begin(), mgIt = muscleGroups.begin(); vIt < values.end(); ++vIt, ++mgIt)
			if (*vIt == currentStrengthCoefficient) {
				found = true;
				mgIt->push_back(currentMuscle);
			}

		if (!found) {
			values.push_back(currentStrengthCoefficient);
			muscleGroups.push_back(vector<int>());
			muscleGroups.at(muscleGroups.size() - 1).push_back(currentMuscle);
		}
	}
}

//TODO: brrr.. questa e' da sistemare
template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getGroupMusclesBasedOnStrengthCoefficientsFilteredByMusclesIndexList(vector<double>& values,
	vector< vector<int> >& muscleGroups,
	const vector<unsigned>& musclesIndexList) {
	getGroupMusclesBasedOnStrengthCoefficients(values, muscleGroups);
	vector< vector <int> > refinedMuscleGroups;
	vector<double> refinedValues;
	vector<bool> checkedMuscle(musclesIndexList.size(), false);

	for (int i = 0; i < musclesIndexList.size(); ++i) {
		if (!checkedMuscle.at(i)) {
			for (int mgIdx = 0; mgIdx < muscleGroups.size(); ++mgIdx) {
				vector<int>::iterator it;
				it = std::find(muscleGroups.at(mgIdx).begin(), muscleGroups.at(mgIdx).end(), musclesIndexList.at(i));
				if (it != muscleGroups.at(mgIdx).end()) {
					refinedMuscleGroups.push_back(muscleGroups.at(mgIdx));
					refinedValues.push_back(values.at(mgIdx));
					//then check as used the other muscles in the group
					for (int muscleIdx = 0; muscleIdx < muscleGroups.at(mgIdx).size(); ++muscleIdx) {
						for (int j = 0; j < musclesIndexList.size(); ++j)
							if (muscleGroups.at(mgIdx).at(muscleIdx) == musclesIndexList.at(j))
								checkedMuscle.at(j) = true;
					}
				}
			}
		}
	}
	//copy refinedMuscleGroups to muscleGroups
	muscleGroups.clear();
	muscleGroups = refinedMuscleGroups;
	values.clear();
	values = refinedValues;

#ifdef DEBUG
	cout << "Muscle Groups:\n";
	for (int i = 0; i < muscleGroups.size(); ++i)
	{
		for (int j = 0; j < muscleGroups.at(i).size(); ++j)
			cout << muscleGroups.at(i).at(j) << " ";
		cout << endl;
	}
#endif
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getShapeFactorsFilteredByMusclesList(std::vector<double>& shapeFactors, const std::vector<std::string>& musclesList) const
{
	shapeFactors.clear();
	shapeFactors.reserve(musclesList.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
	{
		for (std::vector<std::string>::const_iterator itString = musclesList.begin(); itString != musclesList.end(); itString++)
			if (muscleIt->compareMusclesId(*itString))
			{
				shapeFactors.push_back(muscleIt->getShapeFactor());
				break;
			}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getOptimalFiberLengthsFilteredByMusclesList(std::vector<double>& optimalFiberLengths, const std::vector<std::string>& musclesList) const
{
	optimalFiberLengths.clear();
	optimalFiberLengths.reserve(musclesList.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
	{
		for (std::vector<std::string>::const_iterator itString = musclesList.begin(); itString != musclesList.end(); itString++)
			if (muscleIt->compareMusclesId(*itString))
			{
				optimalFiberLengths.push_back(muscleIt->getOptimalFiberLength());
				break;
			}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getTendonSlackLengthsFilteredByMusclesList(std::vector<double>& tendonSlackLengths, const std::vector<std::string>& musclesList) const
{
	tendonSlackLengths.clear();
	tendonSlackLengths.reserve(musclesList.size());
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
	{
		for (std::vector<std::string>::const_iterator itString = musclesList.begin(); itString != musclesList.end(); itString++)
			if (muscleIt->compareMusclesId(*itString))
			{
				tendonSlackLengths.push_back(muscleIt->getTendonSlackLength());
				break;
			}
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMuscle(MTUtype& muscle,
	const string& muscleName) {
	for (int currentMuscle = 0; currentMuscle < muscles_.size(); ++currentMuscle) {
		if (muscles_.at(currentMuscle).compareMusclesId(muscleName)) {
			muscle = muscles_.at(currentMuscle);
			return;
		}
	}

	cout << "NMSmodel::getMuscle: did not found: " << muscleName << endl;
	exit(EXIT_FAILURE);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
const MTU<Activation, Tendon, Curve<mode> >& NMSmodel<Activation, Tendon, mode>::getMuscleConst(const std::string& muscleName) const
{
	for (int currentMuscle = 0; currentMuscle < muscles_.size(); ++currentMuscle)
		if (muscles_.at(currentMuscle).compareMusclesId(muscleName))
			return muscles_.at(currentMuscle);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setStrengthCoefficientsBasedOnGroups(const vector<double>& values,
	const vector< vector<int> >& muscleGroups) {
	for (unsigned int i = 0; i < values.size(); ++i)
		for (unsigned int j = 0; j < muscleGroups.at(i).size(); ++j)
			muscles_.at(muscleGroups.at(i).at(j)).setStrengthCoefficient(values.at(i));
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setShapeFactor(double shapeFactor) {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		muscleIt->setShapeFactor(shapeFactor);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setShapeFactors(const vector<double>& shapeFactors){
	vectorMTUitr muscleIt = muscles_.begin();
	vector<double>::const_iterator shapeFactorsIt = shapeFactors.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt, ++shapeFactorsIt)
		muscleIt->setShapeFactor(*shapeFactorsIt);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setC1(double C1) {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		muscleIt->setC1(C1);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setC1Coefficients(const vector<double>& c1Coefficients){
	vectorMTUitr muscleIt = muscles_.begin();
	vector<double>::const_iterator c1CoefficientsIt = c1Coefficients.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt, ++c1CoefficientsIt)
		muscleIt->setC1(*c1CoefficientsIt);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setC2(double C2){
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		muscleIt->setC2(C2);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setC2Coefficients(const vector<double>& c2Coefficients){
	vectorMTUitr muscleIt = muscles_.begin();
	vector<double>::const_iterator c2CoefficientsIt = c2Coefficients.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt, ++c2CoefficientsIt)
		muscleIt->setC2(*c2CoefficientsIt);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setTendonSlackLengths(const vector<double>& tendonSlackLengths){
	vectorMTUitr muscleIt = muscles_.begin();
	vector<double>::const_iterator tendonSlackLengthIt = tendonSlackLengths.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt, ++tendonSlackLengthIt)
		muscleIt->setTendonSlackLength(*tendonSlackLengthIt);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setOptimalFiberLengths(const std::vector<double>& optimalFiberLengths)
{
	vectorMTUitr muscleIt = muscles_.begin();
	vector<double>::const_iterator optimalFiberLengthsIt = optimalFiberLengths.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt, ++optimalFiberLengthsIt)
		muscleIt->setOptimalFiberLengths(*optimalFiberLengthsIt);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::resetFibreLengthTraces() {
	vectorMTUitr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		muscleIt->resetFibreLengthTrace();
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::resetFibreLengthTraces(const vector<unsigned>& selectedMusclesIndex) {
	vectorMTUitr muscleIt;
	vector<unsigned>::const_iterator it = selectedMusclesIndex.begin();
	for (it; it != selectedMusclesIndex.end(); ++it) {
		muscleIt = muscles_.begin() + *it;
		muscleIt->resetFibreLengthTrace();
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMomentArmsOnDof(vector<double>& momentArms, unsigned whichDof) const {
	dofs_.at(whichDof).getMomentArms(momentArms);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double NMSmodel<Activation, Tendon, mode>::getMusclePenalty() const {
	double penalty = 0.;
	vectorMTUconstItr muscleIt = muscles_.begin();
	for (muscleIt = muscles_.begin(); muscleIt < muscles_.end(); ++muscleIt)
		penalty += muscleIt->getPenalty();
	return penalty;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double NMSmodel<Activation, Tendon, mode>::getMusclePenalty(vector<unsigned>& musclesIndexList) const {
	double penalty = 0.;
	unsigned int mILi = 0;
	for (unsigned int i = 0; i < muscles_.size() && mILi < musclesIndexList.size(); ++i)
		if (i == musclesIndexList.at(mILi)) {
			penalty += muscles_.at(i).getPenalty();
			++mILi;
		}
	return penalty;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclesParameters(vector<MuscleParameters>& parameters) {
	parameters.clear();
	parameters.resize(muscles_.size());

	for (unsigned int i = 0; i < muscles_.size(); ++i) {
		parameters.at(i).setC1(muscles_.at(i).getC1());
		parameters.at(i).setC2(muscles_.at(i).getC2());
		parameters.at(i).setShapeFactor(muscles_.at(i).getShapeFactor());
		parameters.at(i).setOptimalFiberLength(muscles_.at(i).getOptimalFiberLength());
		parameters.at(i).setPennationAngle(muscles_.at(i).getPennationAngle());
		parameters.at(i).setTendonSlackLength(muscles_.at(i).getTendonSlackLength());
		parameters.at(i).setMaxIsometricForce(muscles_.at(i).getMaxIsometricForce());
		parameters.at(i).setStrengthCoefficient(muscles_.at(i).getStrengthCoefficient());
	}
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::setMusclesNamesOnChannel(const std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel)
{
	musclesNamesOnChannel_ = musclesNamesOnChannel;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
void NMSmodel<Activation, Tendon, mode>::getMusclesNamesOnChannel(std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel) const
{
	musclesNamesOnChannel = musclesNamesOnChannel_;
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
std::shared_ptr<NMSModelBase> NMSmodel<Activation, Tendon, mode>::clone(void) const{
	return std::make_shared<NMSmodel<Activation, Tendon, mode>>(*this);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
std::ostream& operator<< (std::ostream& output, const NMSmodel<Activation, Tendon, mode>& b) {
	output << "Current NMSmodel has " << b.muscles_.size() << " muscles:\n";
	for (typename std::vector< MTU<Activation, Tendon, Curve<mode> > >::const_iterator m = b.muscles_.begin();
		m != b.muscles_.end(); ++m) {
		output << "--------- MUSCLE -----------\n";
		output << *m << std::endl;
	}
	output << "and " << b.dofs_.size() << " DoF:\n";
	for (typename vector< DoF<Activation, Tendon, mode> >::const_iterator dof = b.dofs_.begin();
		dof != b.dofs_.end(); ++dof) {
		output << "--------- DOF -----------\n";
		output << *dof << std::endl;
	}
	return output;
}

// The subject object returns data with vector, and the name of the muscle or corresponding dof is in another vector.
// This function collects data from those vector from a string tag
double getValueFromVectorPair(const std::vector<std::string>& nameVector, const std::vector<double>& valueVector, const std::string& tagName){
	auto it = std::find(nameVector.begin(), nameVector.end(), tagName);
	if(it == nameVector.end())
		throw std::runtime_error("Tag " + tagName + "not existent");
	int idx = (int)std::distance(nameVector.begin(), it);;
	return valueVector.at(idx);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double  NMSmodel<Activation, Tendon, mode>::getDofTorque(const std::string& channelName) const{
	std::vector<double> torques;
	this->getTorques(torques);
	return getValueFromVectorPair(this->dofNames_, torques, channelName);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double  NMSmodel<Activation, Tendon, mode>::getMuscleForce(const std::string& channelName) const{
	std::vector<double> muscleForces;
	this->getMuscleForces(muscleForces);
	return getValueFromVectorPair(this->muscleNames_, muscleForces, channelName);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double  NMSmodel<Activation, Tendon, mode>::getMuscleFiberLength(const std::string& channelName) const{
	std::vector<double> fiberLengths;
	this->getFiberLengths(fiberLengths);
	return getValueFromVectorPair(this->muscleNames_, fiberLengths, channelName);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double  NMSmodel<Activation, Tendon, mode>::getMuscleFiberVelocity(const std::string& channelName) const{
	std::vector<double> fiberVelocities;
	this->getFiberVelocities(fiberVelocities);
	return getValueFromVectorPair(this->muscleNames_, fiberVelocities, channelName);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double NMSmodel<Activation, Tendon, mode>::getNormMuscleFiberLength(const std::string& channelName) const{
	std::vector<double> normFiberLengths;
	this->getNormFiberLengths(normFiberLengths);
	return getValueFromVectorPair(this->muscleNames_, normFiberLengths, channelName);
}

template <typename Activation, typename Tendon, CurveMode::Mode mode>
double NMSmodel<Activation, Tendon, mode>::getNormMuscleFiberVelocity(const std::string& channelName) const{
	std::vector<double> normFiberVelocities;
	this->getNormFiberVelocities(normFiberVelocities);
	return getValueFromVectorPair(this->muscleNames_, normFiberVelocities, channelName);
}

#include "NMSmodelPolicyTemplates.h"