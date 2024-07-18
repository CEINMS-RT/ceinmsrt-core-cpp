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


#ifndef NMSmodel_h
#define NMSmodel_h

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <memory>
#include "MTU.h"
#include "DoF.h"
#include "MuscleParameters.h"
#include "SetupDataStructure.h"
#include <boost/timer/timer.hpp>

class NMSModelBase{
    public:
    virtual void getMusclesParameters(vector<MuscleParameters>& parameters) = 0;
    virtual void resetFibreLengthTraces() = 0;
    virtual void resetFibreLengthTraces(const std::vector<unsigned>& selectedMusclesIndex) = 0;
    virtual void setTime(const double& time) = 0;
    virtual void updateFibreLengths_OFFLINEPREP() = 0;
    virtual void updateFibreLengths_OFFLINEPREP(const std::vector<unsigned>& selectedMusclesIndex) = 0;
    virtual void updateFibreLengthTraces() = 0;
    virtual void updateFibreLengthTraces(const std::vector<unsigned>& selectedMusclesIndex) = 0;  
    virtual void setMuscleForces(const std::vector<double>& muscleTendonForces) = 0;
    virtual void setEmgs(const std::vector<double>& currentEMGData) = 0;
    virtual void setEmgsSelective(const std::vector<double>& currentEMGData, const std::vector<unsigned>& selectedMusclesIndex) = 0;
    virtual void setMomentArms(const std::vector<double>& currentMaData, unsigned whichDof) = 0;

    virtual void updateState() = 0;
    virtual void updateState(const std::vector<unsigned>& selectedMusclesIndex) = 0;
    virtual void updateState_HYBRID() = 0;
    virtual void updateState_HYBRID(const std::vector<unsigned>& selectedMusclesIndex) = 0;

    virtual void pushState() = 0;
    virtual void pushState(const std::vector<unsigned>& selectedMusclesIndex) = 0;


    virtual void getEmgs(std::vector<double>& currentEMGData) const = 0;
	virtual void getstrengthCoefficientWithMaxIsometrics(std::vector<double>& isometricMuscleForce) const = 0;
    virtual void getPastEmgs(std::vector<double>& pastEMGData) const = 0;
    virtual void getActivations(std::vector<double>& activations) const = 0;
    virtual void getNeuralActivations( std::vector<double>& neuralActivations) const = 0;
    virtual void getFiberLengths(std::vector<double>& fiberLengths) const = 0;
	virtual void getNormFiberLengths(std::vector<double>& normFiberLengths) const = 0;
    virtual void getFiberVelocities(std::vector<double>& fiberVelocities) const = 0;
	virtual void getNormFiberVelocities(std::vector<double>& normFiberVelocities) const = 0;
	virtual void getNormFiberVelocitiesFilteredByMusclesList(std::vector<double>& normFiberVelocities, const std::vector<std::string>& musclesList) const = 0;
    virtual void getMuscleForces(std::vector<double>& muscleForces) const = 0;
    virtual void getTorques(std::vector<double>& torques) const = 0;
	virtual void getPennationAngle(std::vector<double>& pennationAngle) const = 0;
	virtual void getPennationAngleInst(std::vector<double>& pennationAngle) const = 0;
	virtual void getTendonLength(std::vector<double>& TendonLength) const = 0;
	virtual void getMuscleTendonLengths(std::vector<double>& currentLmtData) const = 0;
	virtual void getMuscleTendonLengthsFilteredByMusclesList(std::vector<double>& currentLmtData, const std::vector<std::string>& musclesList) const = 0;
	virtual void getMuscleForcesFilteredByMusclesList(std::vector<double>& muscleForces, const std::vector<std::string>& musclesList) const = 0;

    // Gets data by name
    virtual double getDofTorque(const std::string& channelName) const = 0;
    virtual double getMuscleForce(const std::string& channelName) const = 0;
    virtual double getMuscleFiberLength(const std::string& channelName) const = 0;
    virtual double getMuscleFiberVelocity(const std::string& channelName) const = 0;
    virtual double getNormMuscleFiberLength(const std::string& channelName) const = 0;
    virtual double getNormMuscleFiberVelocity(const std::string& channelName) const = 0;


    virtual void getMuscleNames(std::vector<std::string>& muscleNames) = 0;
    virtual int  getNoMuscles() const = 0;
    virtual void getDoFNames(std::vector<std::string>& dofNames) = 0;
    virtual int  getNoDoF() = 0;

    virtual void setMuscleTendonLengths(const std::vector<double>& currentLmtData) = 0;
    virtual void setMuscleTendonLengthsSelective(const std::vector<double>& currentLmtData, const std::vector<unsigned>& selectedMusclesIndex) = 0;   
    virtual void setTime_emgs_updateActivations_pushState_selective(double time, const std::vector<double>& currentEMGData, const std::vector<unsigned>& selectedMusclesIndex) = 0;

    virtual void setMusclesNamesOnChannel(const std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel) = 0;
    virtual void getMusclesNamesOnChannel(std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel) const = 0;

    virtual std::shared_ptr<NMSModelBase> clone(void) const = 0;

};

template <typename Activation, typename Tendon, CurveMode::Mode mode>
class NMSmodel;

template <typename Activation, typename Tendon, CurveMode::Mode mode>
std::ostream& operator<< (std::ostream& output, const NMSmodel<Activation, Tendon, mode>& b);


template <typename Activation, typename Tendon, CurveMode::Mode mode>
class NMSmodel : public NMSModelBase
{
public:
    
    typedef Activation ActivationT;
    typedef Tendon TendonT;
    typedef MTU<Activation, Tendon, Curve<mode> > MTUtype;
    typedef DoF<Activation, Tendon, mode> DoFtype;
    typedef typename std::vector<MTUtype>::iterator vectorMTUitr;
    typedef typename std::vector<DoFtype>::iterator vectorDoFitr;
    typedef typename std::vector<MTUtype>::const_iterator vectorMTUconstItr;   
    typedef typename std::vector<DoFtype>::const_iterator vectorDoFconstItr;
     
    NMSmodel() {}
    //NMSmodel(const NMSmodel& orig) { std::cout << "YOU SHOULD NOT BE HERE!"; exit(EXIT_FAILURE);}
	NMSmodel(const NMSmodel& orig) {
		muscles_ = orig.muscles_;
		dofs_ = orig.dofs_;
		muscleNames_ = orig.muscleNames_;
		dofNames_ = orig.dofNames_;
		musclesNamesOnChannel_ = orig.musclesNamesOnChannel_;
	}

    virtual ~NMSmodel() {}
    bool haveThisMuscle(const std::string& currentMuscle, vectorMTUitr& found);
    bool haveTheseMuscles(const std::vector<std::string>& musclesNames);
    bool compareMusclesNames(const std::vector<std::string>& muscleNames) const;
    bool compareMusclesNamesOnDoF (const std::vector<std::string>& muscleNames, int whichDoF) const;

    void setTime(const double& time);
    void setEmgs(const std::vector<double>& currentEMGData);
    void setEmgsSelective(const std::vector<double>& currentEMGData, const std::vector<unsigned>& selectedMusclesIndex);
    void setTime_emgs_updateActivations_pushState_selective(double time, const std::vector<double>& currentEMGData, const std::vector<unsigned>& selectedMusclesIndex);
    void setMuscleTendonLengths(const std::vector<double>& currentLmtData);
    void setMuscleTendonLengthsSelective(const std::vector<double>& currentLmtData, const std::vector<unsigned>& selectedMusclesIndex);   
    void setMomentArms(const std::vector<double>& currentMaData, unsigned whichDof);
    void updateState();
    void updateState(const std::vector<unsigned>& selectedMusclesIndex);
    void updateState_HYBRID();
    void updateState_HYBRID(const std::vector<unsigned>& selectedMusclesIndex);
    void updateActivations();
    void updateActivations(const std::vector<unsigned>& selectedMusclesIndex);
    void updateFibreLengthsAndVelocities();
    void updateFibreLengthsAndVelocities(const std::vector<unsigned>& selectedMusclesIndex);
    void updateFibreLengthsAndVelocities_HYBRID();
    void updateFibreLengthsAndVelocities_HYBRID(const std::vector<unsigned>& selectedMusclesIndex);
    void updateFibreLengths_OFFLINEPREP();
    void updateFibreLengths_OFFLINEPREP(const std::vector<unsigned>& selectedMusclesIndex);
    void updateMuscleForces();
    void updateMuscleForces(const std::vector<unsigned>& selectedMusclesIndex);
    void updateTorques();
    void pushState();
    void pushState(const std::vector<unsigned>& selectedMusclesIndex);
    void updateFibreLengthTraces();
    void updateFibreLengthTraces(const std::vector<unsigned>& selectedMusclesIndex);  
    void resetFibreLengthTraces();
    void resetFibreLengthTraces(const std::vector<unsigned>& selectedMusclesIndex);
    

    // called on model data initialization
    void addMuscle(const MTUtype& muscle);
    void addDoF(const DoFtype& dof);

    // used for optimization plugin to consider only certain DOFs
    void eraseUnusedDofs(const std::vector<std::string>& dofNamesUsed, std::vector<unsigned int>& dofsNotErased);

    // called on model data initialization or in model calibration 
    void setStrengthCoefficientsBasedOnGroups(
                const std::vector<double>& values, 
                const std::vector< std::vector< int > >& muscleGroups);
    void setShapeFactor(double shapeFactor);
    void setShapeFactors(const std::vector<double>& shapeFactors);
    void setC1(double C1);
    void setC1Coefficients(const std::vector<double>& c1Coefficients);
    void setC2(double C2);
    void setC2Coefficients(const std::vector<double>& c2Coefficients);
    void setTendonSlackLengths(const std::vector<double>& tendonSlackLengths);
    void setActivations(const std::vector<double>& activations);
    void setOptimalFiberLengths(const std::vector<double>& optimalFiberLengths);
    
    // called	for control purpose
    void getMuscleNames(std::vector<std::string>& muscleNames) { muscleNames = muscleNames_; }
    int  getNoMuscles() const {return (int)muscles_.size();}
    void getDoFNames(std::vector<std::string>& dofNames) { dofNames = dofNames_; }
    int  getNoDoF() {return (int)dofs_.size();}
    
    // called for get computed data
	void getEmgs(std::vector<double>& currentEMGData) const;
	void getstrengthCoefficientWithMaxIsometrics(std::vector<double>& isometricMuscleForce) const;
    void getPastEmgs(std::vector<double>& pastEMGData) const;
    void getActivations(std::vector<double>& activations) const;
    void getNeuralActivations( std::vector<double>& neuralActivations) const;
    void getFiberLengths(std::vector<double>& fiberLengths) const;
	void getNormFiberLengths(std::vector<double>& normFiberLengths) const;
    void getFiberVelocities(std::vector<double>& fiberVelocities) const;
	void getNormFiberVelocities(std::vector<double>& normFiberVelocities) const;
	void getNormFiberVelocitiesFilteredByMusclesList(std::vector<double>& normFiberVelocities, const std::vector<std::string>& musclesList) const;
    void getMuscleForces(std::vector<double>& muscleForces) const;
    void getMusclePassiveForces(std::vector<double>& muscleForcesPassive) const;
    void getMuscleActiveForces(std::vector<double>& muscleForcesActive) const;
    void getTorques(std::vector<double>& torques) const;
	void getPennationAngle(std::vector<double>& pennationAngle) const;
	void getPennationAngleInst(std::vector<double>& pennationAngle) const;
	void getTendonLength(std::vector<double>& TendonLength) const;
    void getTendonStrain(std::vector<double>& TendonStrain) const;
	void getMuscleTendonLengths(std::vector<double>& currentLmtData) const;
	void getMuscleTendonLengthsFilteredByMusclesList(std::vector<double>& currentLmtData, const std::vector<std::string>& musclesList) const;
	void getMuscleForcesFilteredByMusclesList(std::vector<double>& muscleForces, const std::vector<std::string>& musclesList) const;
	
    // Gets data by name
    double getDofTorque(const std::string& channelName) const;
    double getMuscleForce(const std::string& channelName) const;
    double getMuscleFiberLength(const std::string& channelName) const;
    double getMuscleFiberVelocity(const std::string& channelName) const;
    double getNormMuscleFiberLength(const std::string& channelName) const;
    double getNormMuscleFiberVelocity(const std::string& channelName) const;

    // used during calibration
    void getGroupMusclesBasedOnStrengthCoefficients(std::vector<double>& values, 
                                    std::vector< std::vector< int > >& muscleGroups) const;
    void getGroupMusclesBasedOnStrengthCoefficientsFilteredByMusclesIndexList(std::vector<double>& values, 
                                    std::vector< std::vector< int > >& muscleGroups, const std::vector<unsigned int>& musclesIndexList); 
    void getMusclesIndexOnDof(std::vector<unsigned>& muscleIndex, unsigned whichDof) const;
    void getMusclesIndexFromDofs(std::vector<unsigned int>& musclesIndexList, const std::vector<std::string>& whichDofs);
    void getMusclesIndexFromLastDof(std::vector<unsigned int>& musclesIndexList, const std::vector<std::string>& whichDofs);
    void getMusclesIndexFromMusclesList(std::vector<unsigned>& muscleIndexList, const std::vector<std::string>& musclesList);
    double getShapeFactor() const { return muscles_.at(0).getShapeFactor();}
	void getShapeFactors(std::vector<double>& shapeFactors) const;
	void getShapeFactorsFilteredByMusclesList(std::vector<double>& shapeFactors, const std::vector<std::string>& musclesList) const;
    double getC1() const { return muscles_.at(0).getC1();}
    void getC1Coefficients(std::vector<double>& c1Coefficients) const;
    double getC2() const { return muscles_.at(0).getC2();}
    void getC2Coefficients(std::vector<double>& c2Coefficients) const;  
	void getTendonSlackLengths(std::vector<double>& tendonSlackLengths) const;
	void getTendonSlackLengthsFilteredByMusclesList(std::vector<double>& tendonSlackLengths, const std::vector<std::string>& musclesList) const;
	void getOptimalFiberLengths(std::vector<double>& optimalFiberLengths) const;
	void getOptimalFiberLengthsFilteredByMusclesList(std::vector<double>& optimalFiberLengths, const std::vector<std::string>& musclesList) const;

    void getMuscle(MTUtype& muscle, const std::string& muscleName);
    const MTU<Activation, Tendon, Curve<mode> >& getMuscleConst(const std::string& muscleName) const;
    double getMusclePenalty() const;
    double getMusclePenalty(std::vector<unsigned int>& musclesIndexList) const;
    void getMusclesParameters(std::vector< MuscleParameters >& parameters);
    void setMuscleForces(const std::vector<double>& muscleTendonForces);
    void getMomentArmsOnDof(std::vector<double>& momentArms, unsigned whichDof) const;
    void setMusclesNamesOnChannel(const std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel);
    void getMusclesNamesOnChannel(std::map<std::string, std::vector <std::string> >& musclesNamesOnChannel) const;

    std::shared_ptr<NMSModelBase> clone(void) const;

    friend class SetupDataStructure<NMSmodel<Activation, Tendon, mode>, Curve<mode> >;
    friend std::ostream& operator<< <> (std::ostream& output, const NMSmodel& b);

private:
    std::vector<MTUtype> muscles_;
    std::vector<DoFtype> dofs_;
    std::vector<std::string> muscleNames_;
    std::vector<std::string> dofNames_;
    std::map<std::string, std::vector <std::string> > musclesNamesOnChannel_;
};
/*
template <>
void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateState();

template <>
void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateState(const vector<unsigned>& selectedmusclesIndex);

template <>
void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateFibreLengthsAndVelocities();

template <>
void NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Offline> >, CurveMode::Offline >::updateFibreLengthsAndVelocities(const vector<unsigned>& selectedMusclesIndex);*/

#endif	
