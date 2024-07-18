#include "resample.hpp"
#include <iostream>
#include <algorithm>


std::array<std::vector<double>, 2> resample(const std::vector<double>& dataTime, const std::vector<double>& dataValues, const std::vector<double>& newTimevec){
       
    std::vector<double> newTimevecCopy = newTimevec;
    std::vector<double> resampledData;

    if(newTimevecCopy.front() < dataTime.front()){
        std::cout << "Warning: New time vector starts before data." << std::endl;
        while(newTimevecCopy.front() < dataTime.front())
            newTimevecCopy.erase(newTimevec.begin());
    }


    if(newTimevecCopy.back() > dataTime.back()){
        std::cout << "Warning: New time vector ends after data." << std::endl;
        while(newTimevecCopy.back() < dataTime.back())
            newTimevecCopy.erase(newTimevec.end());
    }

    // Resample is based simply on the sample of the nearest prior timestamp
    for(const double time : newTimevec){
        auto lower = std::lower_bound(dataTime.begin(), dataTime.end(), time);
        if(*lower == time){ // Same sample is used
            auto distance = std::distance(dataTime.begin(), lower);
            resampledData.push_back(dataValues.at(distance));
        }else{ // previous sample is used
            auto distance = std::distance(dataTime.begin(), lower) - 1;
            resampledData.push_back(dataValues.at(distance));
        }
    }

    std::array<std::vector<double>, 2> returnValue = {newTimevecCopy, resampledData};
    return returnValue;
}

std::array<std::vector<double>, 2> cropData(const std::vector<double>& dataTime, const std::vector<double>& dataValues, double timeStart, double timeEnd){
    std::vector<double> dataTimeCopy = dataTime;
    std::vector<double> dataValuesCopy = dataValues;
    
    while(dataTimeCopy.back() > timeEnd){
        dataTimeCopy.erase(dataTimeCopy.end());
        dataValuesCopy.erase(dataValuesCopy.end());     
    }

    while(dataTimeCopy.front() < timeStart){
        dataTimeCopy.erase(dataTimeCopy.begin());     
        dataValuesCopy.erase(dataValuesCopy.begin());     
    }

    std::array<std::vector<double>, 2> returnValue = {dataTimeCopy, dataValuesCopy};
    return returnValue;
}
