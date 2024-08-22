#define	_USE_MATH_DEFINES

#include <DataSources.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <map>

// Whatever the type of the data source object, it has to contain the following functions:
//  double getMuscleForce(const std::string& muscleName) const
//  double getMuscleFiberLength(const std::string& muscleName) const
//  double getDofAngle(const std::string& dofName) const
//  double getMuscleFiberVelocity(const std::string& muscleName) const
//  double getMuscleActivation(const std::string& muscleName) const

class FileXMLParser{
    // boost::property_tree::ptree _fileTree;
    std::string _executionConfigFilepath;
    std::vector<std::string> _dataSourceFiles; // The Order is TorqueSource, GRF source, acceleration, velocity, position, angle. MIssing data sources are empty strings

public:
    FileXMLParser(const std::string& executionFilename);
    void parseDataSources(std::shared_ptr<DataSources>& dataSources);
    void parseDataSources(const std::vector<std::string>& fileNames, std::shared_ptr<DataSources>& dataSources);
};
