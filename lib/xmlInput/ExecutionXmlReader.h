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

#ifndef ExecutionXmlReader_h
#define ExecutionXmlReader_h

#include <string>
#include <vector>
#include "NMSmodelConfig.h"
#include "execution.hxx"


class ExecutionXmlReader {
    
public:
    ExecutionXmlReader(const std::string& filename);
    NMSModelCfg::RunMode getRunMode() const;
    void getMusclesToPredict(std::vector<std::string>& musclesToPredict);
    void getMusclesToTrack(std::vector<std::string>& musclesToTrack);
    void getHybridWeightings(double& alpha, double& beta, double& gamma);
//     void getDynLib(std::string& EMGDynLib, std::string& angleDynLib) const;
    bool isRealTime();
	
	std::string getNameOfSubject();
	std::string getAnglePlugin();
	std::string getAngleFile();
	std::string getEmgPlugin();
	std::string getEmgFile();
	std::string getComsumerPlugin();
	std::string getAngleAndComsumerPlugin();
	std::string getEmgAndAngleAndComsumerPlugin();
	std::string getComsumerPort();
	std::string getOptimizationPlugin();
	std::string getOptimizationFile();
	std::string getEMGAndAnglePlugin();
	
	bool useOfAnglePlugin();
	bool useOfEmgPlugin();
	bool useOfComsumerPlugin();
	bool useOfAngleAndComsumerPlugin();
	bool useOfEmgAndAngleAndComsumerPlugin();
	bool useOfOptimizationPlugin();
	bool useOfEMGAndAnglePlugin();
	
private:
    void readXml();
    
    unsigned runMode_;
    std::auto_ptr<ExecutionType> executionPointer_;
    bool isRealTime_;
	
};

#endif
