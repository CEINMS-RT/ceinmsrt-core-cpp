// This source code is part of:
//
// "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots".
// Copyright (C) 2024 Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau.
//
// CEINMS-RT is an open source software. Any changes to this code, should be shared back in the open repository: https://github.com/CEINMS-RT. See license as described here: https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/LICENSE.
//
// The methodologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
//
// Massimo Sartori, Mohamed Irfan Refai, Lucas Avanci Gaudio, Christopher Pablo Cop, Donatella Simonetti, Federica Damonte, David G. Lloyd, Claudio Pizzolato, Guillaume Durandau., (2024) "CEINMS-RT: an open-source framework for the continuous neuro-mechanical model-based control of wearable robots. TechRxiv. DOI: 10.36227/techrxiv.173397962.28177284/v1"
//

#include "AngleAndComsumer0.h"

AngleAndComsumer0::AngleAndComsumer0(): timeStamp_(0)
{

}

AngleAndComsumer0::~AngleAndComsumer0()
{

}

const std::map<std::string, double>& AngleAndComsumer0::GetDataMap()
{
	for(std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
		dataAngle_[*it] = 0.;
	
	timeStamp_ += 0.01;
	
	return dataAngle_;
}
		
const std::map<std::string, double>& AngleAndComsumer0::GetDataMapTorque()
{
	for(std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++)
		dataTorque_[*it] = 0.;
	
	return dataTorque_;
}

extern "C" AngleAndComsumerPlugin* create() {
    return new AngleAndComsumer0;
}

extern "C" void destroy(AngleAndComsumerPlugin* p) {
    delete p;
}