include_directories(EMGgenerator)

add_library(FileIO STATIC
        DataFromFile.cpp
        HeaderFile.cpp
        MotHeaderFile.cpp
        DataFromAngleFile.cpp)

target_link_libraries(FileIO
        Boost::system)

add_subdirectory(DataSources)
add_subdirectory(FileIOParser)