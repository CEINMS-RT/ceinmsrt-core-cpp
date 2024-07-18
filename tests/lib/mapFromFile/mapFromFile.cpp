#include <DataFromFile.h>
#include "mapFromFile.hpp"

std::map<std::string, std::vector<double>> mapFromFile(const std::string& fileName){
    DataFromFile data(fileName);
    std::map<std::string, std::vector<double>> outMap;
    const auto names = data.getColumnNames();
    data.readNextData(); // First step mandatory before reading data

    outMap.insert({"time", std::vector<double>()});
    for(const auto& name : names){
        outMap.insert({name, std::vector<double>()}); // Initializes vector
    }

    while(data.areStillData()){
        auto values = data.getCurrentData();
        outMap.at("time").push_back(data.getCurrentTime());
        for(int idx = 0; idx < names.size(); idx++){
            outMap.at(names[idx]).push_back(values[idx]);
        }
        data.readNextData();
    }
    return outMap;
}