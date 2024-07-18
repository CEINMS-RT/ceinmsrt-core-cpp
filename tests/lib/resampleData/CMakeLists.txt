
project(ResampleDataLib)

add_library(${PROJECT_NAME} resample.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})