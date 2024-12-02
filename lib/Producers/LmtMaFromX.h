// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software, regulated by the license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots"
//

#ifndef LmtMaFromX_h
#define LmtMaFromX_h

#include <vector>
#include <string>

class LmtMaFromX
{
  public:
    virtual ~LmtMaFromX();
    virtual void operator()() {};
    void pushLmtBack(const std::vector<double>& newLmtToPush);
    void pushMomentArmsBack(const std::vector<double>& newMomentArmsToPush, unsigned int whichDof);
    void updateLmt(const std::vector<double>& currentLmtData, double currentTime);
    void updateMomentArms(const std::vector<double>& currentMomentArmsData, double currentTime, unsigned int whichDof); 
    void getDofNames(std::vector< std::string >& dofNamesFromModel);
    void getMuscleNamesFromShared(std::vector< std::string >& muscleNamesFromModel);
    void getAngleFromShared(std::vector<double>& angleFromQueue);
	void setMusclesNamesOnDof(std::vector<std::vector<std::string> >& musclesNamesOnDof);
	void getSyncGui(bool& gui);
	void getSyncVerbose(int& verbose);
	void AddToTimeConsumeMTU(const double& timeToAdd);
	void AddToNbFrameMTU(const double& timeToAdd);
};

#endif
