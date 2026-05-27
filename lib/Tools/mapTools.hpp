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

#pragma once
#include <vector>
#include <map>
#include <string>

template<typename KeyT, typename valueT> inline
std::vector<valueT> mapToVector(const std::map<KeyT, valueT>& map);


std::map<std::string, double> vectorToMap(const std::vector<std::string>& keys, const std::vector<double>& values);


//This stuff can become a template
std::vector<std::string> mapToKeys(const std::map<std::string, double>& map);
std::vector<std::string> mapToKeys(const std::map<std::string, std::vector<std::string>>& map);
std::vector<std::string> mapToKeys(const std::map<std::string, std::vector<double>>& map);

// template<typename KeyT, typename valueT> inline
// std::vector<valueT> mapToKeys(const std::map<KeyT, valueT>& map);
