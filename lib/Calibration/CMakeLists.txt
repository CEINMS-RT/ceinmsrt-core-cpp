include_directories(
        ${CMAKE_SOURCE_DIR}/lib/xmlInput
        ${CMAKE_SOURCE_DIR}/lib/NMSmodel
        ${CMAKE_SOURCE_DIR}/lib/Curve
        ${CMAKE_SOURCE_DIR}/lib/FileIO
        ${CMAKE_SOURCE_DIR}/lib/Calibration)

add_library(Calibration_rand STATIC
        random-gen.cpp)

add_library(SyncToolsCal STATIC
        SyncToolsCal.cpp
        ../Producers/Semaphore.cpp)

target_link_libraries(SyncToolsCal
        Boost::system)