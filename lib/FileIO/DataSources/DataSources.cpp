#include "DataSources.hpp"
#include <cstdlib>
#include <algorithm>
#include <SyncTools.h>



void DataSources::setAngleSource(const std::string& fileName){
    if(fileName != ""){
        this->_angleSource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_angleSource.available = false;
    }
}

void DataSources::setEMGSource(const std::string& fileName){
    if(fileName != ""){
        this->_emgSource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_emgSource.available = false;
    }
}

void DataSources::setTorqueSource(const std::string& fileName){
    if(fileName != ""){
        this->_torqueSource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_torqueSource.available = false;
    }
}

void DataSources::setGRFSource
(const std::string& fileName){
        if(fileName != ""){
        this->_grfSource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_grfSource.available = false;
    }
}

void DataSources::setAccelerationSource(const std::string& fileName){
        if(fileName != ""){
        this->_accelerationSource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_accelerationSource.available = false;
    }
}

void DataSources::setVelocitySource(const std::string& fileName){
    if(fileName != ""){
        this->_velocitySource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_velocitySource.available = false;
    }
}

void DataSources::setPositionSource(const std::string& fileName){
    if(fileName != ""){
        this->_positionSource = {true, std::make_shared<DataFromFile>(fileName)};
    } else{
        this->_positionSource.available = false;
    }
}

void DataSources::step(void){
    // static bool firstStep = true;
    // if(firstStep){
    //     firstStep = false; // Does not step if it is the first line
    //     return;
    // }

    if(!this->dataStillAvailable()){
        COUT << "End of input files reached at (angle) time " << this->_angleSource.hdl->getCurrentTime() << std::endl;
		// InterThread::setEndThread(true);
        exit(EXIT_SUCCESS);
        return;
    }

    if(this->_angleSource.available)
        this->_angleSource.hdl->readNextData();
    
    this->_currTime =  this->_angleSource.hdl->getCurrentTime();

    if(this->_emgSource.available)
        this->_emgSource.hdl->stepToTime(this->_currTime);

    if(this->_torqueSource.available)
        this->_torqueSource.hdl->stepToTime(this->_currTime);

    if(this->_grfSource.available)
        this->_grfSource.hdl->stepToTime(this->_currTime);

    if(this->_accelerationSource.available)
        this->_accelerationSource.hdl->stepToTime(this->_currTime);

    if(this->_velocitySource.available)
        this->_velocitySource.hdl->stepToTime(this->_currTime);

    if(this->_positionSource.available)
        this->_positionSource.hdl->stepToTime(this->_currTime);

}

void DataSources::stepToTime(double time){

    if(time < this->_currTime){
        if(this->_angleSource.available)
            this->_angleSource.hdl->resetFile();

        if(this->_emgSource.available)
            this->_emgSource.hdl->resetFile();
    
        if(this->_torqueSource.available)
            this->_torqueSource.hdl->resetFile();

        if(this->_grfSource.available)
            this->_grfSource.hdl->resetFile();

        if(this->_accelerationSource.available)
            this->_accelerationSource.hdl->resetFile();

        if(this->_velocitySource.available)
            this->_velocitySource.hdl->resetFile();

        if(this->_positionSource.available)
            this->_positionSource.hdl->resetFile();
    }

    if(this->_angleSource.available)
        this->_angleSource.hdl->stepToTime(time);

    if(this->_emgSource.available)
        this->_emgSource.hdl->stepToTime(time);
    
    if(this->_torqueSource.available)
        this->_torqueSource.hdl->stepToTime(time);

    if(this->_grfSource.available)
        this->_grfSource.hdl->stepToTime(time);

    if(this->_accelerationSource.available)
        this->_accelerationSource.hdl->stepToTime(time);

    if(this->_velocitySource.available)
        this->_velocitySource.hdl->stepToTime(time);

    if(this->_positionSource.available)
        this->_positionSource.hdl->stepToTime(time);

    this->_currTime =  this->_angleSource.hdl->getCurrentTime();

}



std::map<std::string, double>& DataSources::buildMap(std::map<std::string, double>& inputMap, const std::vector<std::string>& names, const std::vector<double>& data){
    if(names.size() != data.size()){
        std::cout << "Data labels and data size do not match on input file." << std::endl;
        throw std::runtime_error("Data labels and data size do not match on input file.");
    }
    for(int idx = 0; idx < names.size(); idx++){
        inputMap[names[idx]] = data[idx];
    }
    return inputMap;

}


    const std::map<std::string, double>& DataSources::getAngle(void){
        if(!this->_angleSource.available)
            return this->_angleMap; // Empty map
        return this->buildMap(this->_angleMap, this->_angleSource.hdl->getColumnNames(), this->_angleSource.hdl->getCurrentData());
    }

    const std::map<std::string, double>& DataSources::getEMG(void){
        if(!this->_emgSource.available)
            return this->_emgMap; // Empty map
        return this->buildMap(this->_emgMap, this->_emgSource.hdl->getColumnNames(), this->_emgSource.hdl->getCurrentData());
    }

    const std::map<std::string, double>& DataSources::getTorque(void){
        if(!this->_torqueSource.available)
            return this->_torqueMap; // Empty map
        return this->buildMap(this->_torqueMap, this->_torqueSource.hdl->getColumnNames(), this->_torqueSource.hdl->getCurrentData());
    }

    const std::map<std::string, double>& DataSources::getGRF(void){
        if(!this->_grfSource.available)
            return this->_grfMap; // Empty map
        return this->buildMap(this->_grfMap, this->_grfSource.hdl->getColumnNames(), this->_grfSource.hdl->getCurrentData());
    }

    const std::map<std::string, double>& DataSources::getAcceleration(void){
        if(!this->_accelerationSource.available)
            return this->_accelerationMap; // Empty map
        return this->buildMap(this->_accelerationMap, this->_accelerationSource.hdl->getColumnNames(), this->_accelerationSource.hdl->getCurrentData());
    }

    const std::map<std::string, double>& DataSources::getVelocity(void){
        if(!this->_velocitySource.available)
            return this->_velocityMap; // Empty map
        return this->buildMap(this->_velocityMap, this->_velocitySource.hdl->getColumnNames(), this->_velocitySource.hdl->getCurrentData());
    }

    const std::map<std::string, double>& DataSources::getPosition(void){
        if(!this->_positionSource.available)
            return this->_positionMap; // Empty map
        return this->buildMap(this->_positionMap, this->_positionSource.hdl->getColumnNames(), this->_positionSource.hdl->getCurrentData());
    }    

const double& DataSources::currTimestamp(void){
    return this->_currTime;
}


bool DataSources::dataStillAvailable(void){

    if( !this->_angleSource.available  &&
        !this->_emgSource.available &&
        !this->_grfSource.available &&
        !this->_torqueSource.available &&
        !this->_positionSource.available &&
        !this->_velocitySource.available &&
        !this->_accelerationSource.available )
    return false;

    bool returnValue = true;
    if(this->_angleSource.available) returnValue = returnValue & this->_angleSource.hdl->areStillData();
    if(this->_emgSource.available) returnValue = returnValue & this->_emgSource.hdl->areStillData();
    if(this->_grfSource.available) returnValue = returnValue & this->_grfSource.hdl->areStillData();
    if(this->_torqueSource.available) returnValue = returnValue & this->_torqueSource.hdl->areStillData();
    if(this->_positionSource.available) returnValue = returnValue & this->_positionSource.hdl->areStillData();
    if(this->_velocitySource.available) returnValue = returnValue & this->_velocitySource.hdl->areStillData();
    if(this->_accelerationSource.available) returnValue = returnValue & this->_accelerationSource.hdl->areStillData();


    return returnValue;

    // return (    (this->_angleSource.hdl->areStillData() || !this->_angleSource.available)  &&
    //             (this->_emgSource.hdl->areStillData() ||  !this->_emgSource.available) &&
    //             (this->_grfSource.hdl->areStillData() || !this->_grfSource.available) &&
    //             (this->_torqueSource.hdl->areStillData() || !this->_torqueSource.available) &&
    //             (this->_positionSource.hdl->areStillData() || !this->_positionSource.available) &&
    //             (this->_velocitySource.hdl->areStillData() || !this->_velocitySource.available) &&
    //             (this->_accelerationSource.hdl->areStillData() || !this->_accelerationSource.available) );
}


double DataSources::getAngleTime(void){
    if(this->_angleSource.available)
        return this->_angleSource.hdl->getCurrentTime();
    return -1;
}   

double DataSources::getEMGTime(void){
    if(this->_emgSource.available)
        return this->_emgSource.hdl->getCurrentTime();
    return -1;
}   

double DataSources::getTorqueTime(void){
    if(this->_torqueSource.available)
        return this->_torqueSource.hdl->getCurrentTime();
    return -1;
}   

double DataSources::getGRFTime(void){
    if(this->_grfSource.available)
        return this->_grfSource.hdl->getCurrentTime();
    return -1;
}   

double DataSources::getAccelerationTime(void){
    if(this->_accelerationSource.available)
        return this->_accelerationSource.hdl->getCurrentTime();
    return -1;
}   

double DataSources::getVelocityTime(void){
    if(this->_velocitySource.available)
        return this->_velocitySource.hdl->getCurrentTime();
    return -1;
}   

double DataSources::getPositionTime(void){
    if(this->_positionSource.available)
        return this->_positionSource.hdl->getCurrentTime();
    return -1;
}   


double getChannelData(const DataSourcesStruct& channelStruct, const std::string& channelName, const std::string& dataName){
    if(!channelStruct.available)
        throw std::runtime_error( dataName + " source not available");

    auto columnNames = channelStruct.hdl->getColumnNames();
    auto it = std::find(columnNames.begin(), columnNames.end(), channelName);
    if(it == columnNames.end())
         throw std::runtime_error( dataName +  " channel not available: " + channelName);
    int idx = (int) (it - columnNames.begin());
    return channelStruct.hdl->getCurrentData().at(idx);
}


double DataSources::getDofAngle(const std::string& channelName) const{
    return getChannelData(this->_angleSource, channelName, "Angle");
}

double DataSources::getDofEMG(const std::string& channelName) const{
    return getChannelData(this->_emgSource, channelName, "EMG");
}

double DataSources::getDofTorque(const std::string& channelName) const{
    return getChannelData(this->_torqueSource, channelName, "Torque");
}

double DataSources::getChannelGRF(const std::string& channelName) const{
    return getChannelData(this->_grfSource, channelName, "GRF");
}
double DataSources::getChannelAcceleration(const std::string& channelName) const{
    return getChannelData(this->_accelerationSource, channelName, "Acceleration");
}
double DataSources::getChannelVelocity(const std::string& channelName) const{
    return getChannelData(this->_velocitySource, channelName, "Velocity");
}
double DataSources::getChannelPosition(const std::string& channelName) const{
    return getChannelData(this->_positionSource, channelName, "Position");
}



