add_library(MapTools    
        mapTools.cpp)


target_include_directories(MapTools PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})


# add_library(MapTools INTERFACE)
# target_include_directories(MapTools INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})