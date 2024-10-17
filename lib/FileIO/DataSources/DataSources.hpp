#ifndef DATASOURCES_HPP_
#define DATASOURCES_HPP_
#define	_USE_MATH_DEFINES

#include <string>
#include <DataFromFile.h>
#include <memory>
#include <map>


struct DataSourcesStruct
{
    bool available;
    std::shared_ptr<DataFromFile> hdl;
};

    class DataSources{
    DataSourcesStruct _angleSource;
    DataSourcesStruct _emgSource;
    DataSourcesStruct _torqueSource;
    DataSourcesStruct _grfSource; // Ground Reaction Force
    DataSourcesStruct _accelerationSource;
    DataSourcesStruct _velocitySource;
    DataSourcesStruct _positionSource;

    std::map<std::string, double> _angleMap;
    std::map<std::string, double> _emgMap;
    std::map<std::string, double> _torqueMap;
    std::map<std::string, double> _grfMap;
    std::map<std::string, double> _accelerationMap;
    std::map<std::string, double> _velocityMap;
    std::map<std::string, double> _positionMap;

    std::map<std::string, double>& buildMap(std::map<std::string, double>& inputMap, const std::vector<std::string>& names, const std::vector<double>& data);
    double _currTime;

public:
    DataSources() = default;
    void setAngleSource(const std::string& fileName);
    void setEMGSource(const std::string& fileName);
    void setTorqueSource(const std::string& fileName);
    void setGRFSource(const std::string& fileName);
    void setAccelerationSource(const std::string& fileName);
    void setVelocitySource(const std::string& fileName);
    void setPositionSource(const std::string& fileName);

    const std::map<std::string, double>& getAngle(void);
    const std::map<std::string, double>& getEMG(void);
    const std::map<std::string, double>& getTorque(void);
    const std::map<std::string, double>& getGRF(void);
    const std::map<std::string, double>& getAcceleration(void);
    const std::map<std::string, double>& getVelocity(void);
    const std::map<std::string, double>& getPosition(void);


    double getDofAngle(const std::string& channelName) const;
    double getDofEMG(const std::string& channelName) const;
    double getDofTorque(const std::string& channelName) const;
    double getChannelGRF(const std::string& channelName) const;
    double getChannelAcceleration(const std::string& channelName) const;
    double getChannelVelocity(const std::string& channelName) const;
    double getChannelPosition(const std::string& channelName) const;

    double getAngleTime(void);
    double getEMGTime(void);
    double getTorqueTime(void);
    double getGRFTime(void);
    double getAccelerationTime(void);
    double getVelocityTime(void);
    double getPositionTime(void);

    const double& currTimestamp(void);

    void step(void);
    void stepToTime(double time);

    bool dataStillAvailable(void);

};


#endif