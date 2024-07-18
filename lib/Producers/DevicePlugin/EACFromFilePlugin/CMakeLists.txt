
add_library(PluginEACFromFile SHARED
        EACFromFilePlugin.cpp)

target_include_directories(PluginEACFromFile PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PluginEACFromFile
        Boost::system
        FileIO
        DataSources
        DataFromFileParser)
