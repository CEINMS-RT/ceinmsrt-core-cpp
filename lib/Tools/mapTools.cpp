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
#include <iostream>
#include <array>
#include "mapTools.hpp"

template<typename KeyT, typename valueT> inline
std::vector<valueT> mapToVector(const std::map<KeyT, valueT>& map){
    std::vector<valueT> tempData;
    for(const auto& entry : map){
        tempData.push_back(entry.second);
    }
    return tempData;
}

// Could be a template. Not necessary for now
std::map<std::string, double> vectorToMap(const std::vector<std::string>& keys, const std::vector<double>& values){
    std::map<std::string, double> map;
    if(keys.size() != values.size()){
        std::cout << "Cannot create map from different size vectors" << std::endl;
        throw std::runtime_error("Cannot create map from different size vectors");
    }
    for(int idx = 0; idx < keys.size(); idx++){
        map[keys[idx]] = values[idx];
    }
    return map;
}

// Forces the compilation of the types defined below. That allows us to compile it only in this module and link it to the others.
template std::vector<double> mapToVector(const std::map<std::string, double>& map);
template std::vector<double> mapToVector(const std::map<std::array<std::string, 5>, double>& map); 

std::vector<std::string> mapToKeys(const std::map<std::string, double>& map){
    std::vector<std::string> tempData;
    for(const auto& entry : map){
        tempData.push_back(entry.first);
    }
    return tempData;
}

std::vector<std::string> mapToKeys(const std::map<std::string, std::vector<std::string>>& map){
    std::vector<std::string> tempData;
    for(const auto& entry : map){
        tempData.push_back(entry.first);
    }
    return tempData;
}
std::vector<std::string> mapToKeys(const std::map<std::string, std::vector<double>>& map){
    std::vector<std::string> tempData;
    for(const auto& entry : map){
        tempData.push_back(entry.first);
    }
    return tempData;
}

// template<typename KeyT, typename valueT> inline
// std::vector<KeyT> mapToKeys(const std::map<KeyT, valueT>& map){
//     std::vector<KeyT> tempData;
//     for(const auto& entry : map){
//         tempData.push_back(entry.first);
//     }
//     return tempData;
// }