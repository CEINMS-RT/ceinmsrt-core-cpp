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