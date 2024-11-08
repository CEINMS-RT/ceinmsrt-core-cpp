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
