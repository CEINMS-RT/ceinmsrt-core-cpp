
project(MapFromFileLib)

add_library(${PROJECT_NAME} mapFromFile.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(MapFromFileLib
    FileIO)