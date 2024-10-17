#define BOOST_TEST_MODULE "PluginAAC Interface Tests"
#ifdef __GNUC__
#define BOOST_TEST_DYN_LINK
#endif

#include <boost/test/unit_test.hpp>
#include <boost/dll.hpp>
#include <cstdlib>
#include <compareFiles.hpp>
#include <resample.hpp>
#include <mapFromFile.hpp>
#include <mapTools.hpp>
#include <correlation.hpp>
#include <resample.hpp>
#ifdef __GNUC__
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <boost/filesystem.hpp>
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

// Defines come from CMAKE 
// int main(){
//     return 0;
// }


BOOST_AUTO_TEST_SUITE(AngleAndConsumerTests)
BOOST_AUTO_TEST_CASE(AngleAndConsumerProducer)
{
    #ifdef WIN32
        std::string exeExt = ".exe";
    #else
        std::string exeExt = "";
    #endif

    std::string currentSubfolder;
    #ifdef __GNUC__
        {
            namespace fs = boost::filesystem;        
            currentSubfolder = fs::relative(fs::path(CMAKE_CURRENT_SOURCE_DIR), fs::path(std::string(CMAKE_SOURCE_DIR) + "/tests")).string();
        }
    #else
        currentSubfolder = fs::relative(fs::path(CMAKE_CURRENT_SOURCE_DIR), fs::path(std::string(CMAKE_SOURCE_DIR) + "/tests")).string();
    #endif
    std::string execFile = "configFiles/executionFiles/ExecutionAACAndEMGProducerVirtualSbj04.xml";
    std::string sbjFile = "configFiles/subjectFiles/Subj04AnkleCalibrated.xml";
    std::string expectedResultsFolder = "expectedResults/" + currentSubfolder;
    std::string binFolder = fs::absolute(boost::dll::program_location().parent_path().string()).string() ;
    std::string ceinmsFolder = fs::absolute(boost::dll::program_location().parent_path().string() + "/../../../").string();


    std::string currFolder = fs::current_path().string();
    fs::current_path(ceinmsFolder + "/data/testData");
    currFolder = fs::current_path().string();
    std::string absOutFolder = ceinmsFolder + "output/" + currentSubfolder;
    if(!fs::exists(absOutFolder)){
        if(!fs::create_directories(absOutFolder))
            BOOST_CHECK_MESSAGE( false, "Failed to create output folder");
    }

    std::string relOutFolder;
    #ifdef __GNUC__
        {
            namespace fs = boost::filesystem;        
            relOutFolder = fs::relative(fs::path(absOutFolder), fs::path(currFolder)).string();
        }
    #else
        relOutFolder = fs::relative(fs::path(CMAKE_CURRENT_SOURCE_DIR), fs::path(std::string(CMAKE_SOURCE_DIR) + "/tests")).string();
    #endif



    std::string command = binFolder + "/CEINMS" + exeExt + " -e " + execFile + " -s " + sbjFile + " -r " + relOutFolder;
    // std::string command = binFolder + "/CEINMS" + exeExt;
    int result = std::system(command.c_str());
    if(result)
        BOOST_CHECK_MESSAGE( false, "Execution of CEINMS failed");
    else
        BOOST_TEST(true);



    for (const auto & fileName : fs::directory_iterator(expectedResultsFolder)){
        if(fileName.path().filename().string() == "MTUTiming.csv" || fileName.path().filename().string() == "NMSTimming.csv" || fileName.path().filename().string() == "TotalTimming.csv")
            continue; 

        std::string refFile = expectedResultsFolder + "/" + fileName.path().filename().string();
        std::string outFile = fs::path(absOutFolder + "/" + fileName.path().filename().string()).make_preferred().string();

        std::map<std::string, std::vector<double>> expectedResultsMap = mapFromFile(refFile);
        std::map<std::string, std::vector<double>> outResultsMap = mapFromFile(outFile);

        std::vector<std::string> keyVec = mapToKeys(expectedResultsMap);

        for(const auto& name : keyVec){
            if(name == "time")
                continue;
            
            // Resample to reference data timestamps
            auto resampledData = resample(outResultsMap.at("time"), outResultsMap.at(name), expectedResultsMap.at("time"));
            // Crop reference timestamps to the time that the output actually has data
            auto croppedRefData = cropData(expectedResultsMap.at("time"), expectedResultsMap.at(name), outResultsMap.at("time").front(), outResultsMap.at("time").back());

            double correlationVal = calcCorrelation(resampledData[1], croppedRefData[1]);
            if(correlationVal <= 0.99)
                BOOST_CHECK_MESSAGE( false, fileName.path().filename().string() + " file equivalence failed");

        }
        BOOST_TEST(true);
    }
    BOOST_TEST(true);
}


BOOST_AUTO_TEST_SUITE_END()