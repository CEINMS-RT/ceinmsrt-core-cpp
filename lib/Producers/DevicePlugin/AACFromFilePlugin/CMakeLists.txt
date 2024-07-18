
add_library(PluginAACFromFile SHARED
        AACFromFilePlugin.cpp)

target_include_directories(PluginAACFromFile PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PluginAACFromFile
        Boost::system
        FileIO
        DataSources
        DataFromFileParser)
