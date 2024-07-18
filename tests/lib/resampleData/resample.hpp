#pragma once
#include <vector>
#include <array>

std::array<std::vector<double>, 2> resample(const std::vector<double>& dataTime, const std::vector<double>& dataValues, const std::vector<double>& newTimevec);

std::array<std::vector<double>, 2> cropData(const std::vector<double>& dataTime, const std::vector<double>& dataValues, double timeStart, double timeEnd);
