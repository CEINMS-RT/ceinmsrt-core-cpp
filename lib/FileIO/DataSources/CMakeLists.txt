
add_library(DataSources STATIC
        DataSources.cpp)

target_include_directories(DataSources PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(DataSources
        Boost::system
        FileIO
        Producers)
