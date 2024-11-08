#pragma once 

#include "DataFromFileParser.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <map>
#include <memory>
#include <string>
    FileXMLParser::FileXMLParser(const std::string& executionFilename){
        boost::property_tree::ptree subjectFileTree, executionFileTree;
        try{
            boost::property_tree::read_xml(executionFilename, executionFileTree);
            this->_executionConfigFilepath = executionFilename;
        }catch (std::exception &e){
            std::cout << "Error: " << std::string(e.what()) << "\n";
        }
        auto execution = executionFileTree.get_child("execution");
        auto plugin = execution.get_child("ConsumerPlugin");
        
        this->_dataSourceFiles.push_back(plugin.get<std::string>("TorqueDeviceFile", ""));
        this->_dataSourceFiles.push_back(plugin.get<std::string>("GRFDeviceFile", ""));
        this->_dataSourceFiles.push_back(plugin.get<std::string>("AccelerationDeviceFile", ""));
        this->_dataSourceFiles.push_back(plugin.get<std::string>("VelocityDeviceFile", ""));
        this->_dataSourceFiles.push_back(plugin.get<std::string>("PositionDeviceFile", ""));
        this->_dataSourceFiles.push_back(plugin.get<std::string>("AngleOfflineFile", ""));
        this->_dataSourceFiles.push_back(plugin.get<std::string>("EMGDeviceFile", ""));
    }


// Order:
// [0] Torque
// [1] GRF
// [2] Acceleration (segment accelerations)
// [3] Velocity (segment velocity)
// [4] Position (segment acceleration)
// [5] Angle (DOF angle)
// [6] EMG (channel)
void FileXMLParser::parseDataSources(std::shared_ptr<DataSources>& dataSources){ 
    this->parseDataSources(this->_dataSourceFiles, dataSources);    
}


// Order:
// [0] Torque
// [1] GRF
// [2] Acceleration (segment accelerations)
// [3] Velocity (segment velocity)
// [4] Position (segment acceleration)
// [5] Angle (DOF angle)
// [6] EMG (channel)
void FileXMLParser::parseDataSources(const std::vector<std::string>& fileNames, std::shared_ptr<DataSources>& dataSources){

    dataSources = std::make_shared<DataSources>();

    dataSources->setTorqueSource(fileNames[0]);
    dataSources->setGRFSource(fileNames[1]);
    dataSources->setAccelerationSource(fileNames[2]);
    dataSources->setVelocitySource(fileNames[3]);
    dataSources->setPositionSource(fileNames[4]);
    dataSources->setAngleSource(fileNames[5]);
    dataSources->setEMGSource(fileNames[6]);
    
}