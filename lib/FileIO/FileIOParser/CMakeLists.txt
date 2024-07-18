
add_library(DataFromFileParser STATIC
        DataFromFileParser.cpp)

target_include_directories(DataFromFileParser PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(DataFromFileParser
        Boost::system
        DataSources)
