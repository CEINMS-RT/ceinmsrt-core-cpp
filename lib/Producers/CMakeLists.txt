add_library(PreprocessingEMG
        EMGPreProcessing.cpp)

add_library(PluginEMG0 SHARED
        DevicePlugin/EMG0plugin.cpp)

add_library(PluginEMG1 SHARED
        DevicePlugin/EMG1plugin.cpp)

add_library(PluginAngle0 SHARED
        DevicePlugin/Angle0plugin.cpp)

add_library(PluginAngleAndComsumer0 SHARED
        DevicePlugin/AngleAndComsumer0.cpp)

set(_extra_libs XMLExecutionInterpreter)

if (WIN32)
    set(_extra_libs
            ${_extra_libs}
            ws2_32
            winmm)
endif ()

target_link_libraries(PluginEMG0 ${_extra_libs})
target_link_libraries(PluginAngle0 ${_extra_libs})
target_link_libraries(PluginEMG1 ${_extra_libs})

add_library(PluginEMGFiltFromFile SHARED
        DevicePlugin/EMGFileplugin.cpp)

add_library(PluginEMGFromFile SHARED
        DevicePlugin/EMGFilepluginFilt.cpp)

target_link_libraries(PluginEMGFiltFromFile
        FileIO
        XMLExecutionInterpreter
        XMLEMGInterpreter
        XMLIKIDInterpreter
        PreprocessingEMG)

target_compile_features(PluginEMGFiltFromFile PRIVATE cxx_std_17)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(PluginEMGFiltFromFile stdc++fs)
endif()

target_link_libraries(PluginEMGFromFile
        FileIO
        XMLExecutionInterpreter
	XMLEMGInterpreter
        XMLIKIDInterpreter
        PreprocessingEMG)

add_library(PluginAngleFromFile SHARED
        DevicePlugin/AngleFileplugin.cpp)

target_link_libraries(PluginAngleFromFile
        FileIO)

add_library(PluginAngleAndIDFromFile SHARED
        DevicePlugin/AngleAndIDFileplugin.cpp)

target_link_libraries(PluginAngleAndIDFromFile
        FileIO)
		
		
add_library(DynLib
        FromExtDevice/DynLib.cpp
)

target_link_libraries(DynLib
        XMLExecutionInterpreter
        Boost::chrono
        Boost::timer
        Boost::system)
		
target_compile_features(DynLib PUBLIC cxx_std_17)
target_compile_definitions(DynLib PUBLIC -D_HAS_STD_BYTE=0)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(DynLib stdc++fs)
endif()


add_library(DynLibOptimization
        FromExtDevice/DynLibOptimization.cpp)

target_link_libraries(DynLibOptimization
        XMLExecutionInterpreter
        Boost::chrono
        Boost::timer
        Boost::system)
		
target_compile_features(DynLibOptimization PUBLIC cxx_std_17)
target_compile_definitions(DynLibOptimization PUBLIC -D_HAS_STD_BYTE=0)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(DynLibOptimization stdc++fs)
endif()

add_library(Producers
        EMGFromX.cpp
        ExternalTorqueFromX.cpp
        LmtMaFromX.cpp
        AngleFromX.cpp
        Semaphore.cpp
        SyncTools.cpp
        FromFile/LmtMaFromFile.cpp
        FromExtDevice/AngleFromDevice.cpp
        FromExtDevice/EMGFromDevice.cpp
        FromExtDevice/LmtMaFromMTUSpline.cpp)

target_link_libraries(Producers
        XMLExecutionInterpreter
        FileIO
        MTUSpline
        FileLogger
        DynLibOptimization
        DynLib
        Boost::thread)

target_compile_features(Producers PUBLIC cxx_std_17)
target_compile_definitions(Producers PUBLIC -D_HAS_STD_BYTE=0)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(Producers stdc++fs)
endif()

add_subdirectory(DevicePlugin/EACFromFilePlugin)
add_subdirectory(DevicePlugin/AACFromFilePlugin)
