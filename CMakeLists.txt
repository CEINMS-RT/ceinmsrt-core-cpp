cmake_minimum_required(VERSION 3.10)
project(CEINMS)

cmake_policy(SET CMP0087 NEW)

file(GLOB RESULT ${PROJECT_SOURCE_DIR}/lib/Gui/QCustomPlot/*.txt)
if(RESULT STREQUAL "" )
message("Collecting QCustomPlot. Please wait.")
execute_process(
    COMMAND git submodule update --init lib/Gui/QCustomPlot
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
else()
message("QCustomPlot already cloned. Delete it to re-download.")
endif()

file(GLOB RESULT ${PROJECT_SOURCE_DIR}/lib/tclap/*.txt)
if(RESULT STREQUAL "")
message("Collecting tclap. Please wait.")
execute_process(
    COMMAND git submodule update --init lib/tclap
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
else()
message("tclap already cloned. Delete it to re-download.")
endif()

include(GNUInstallDirs)

option(USE_GUI "Enable the GUI while running CEINMS" ON)
option(USE_OPENSIM "Compile with OpenSim features" ON)
option(OPENSIM4 "Use OpenSim 4 instead of 3" ON)
option(RECORD_IN_FILE "Disabled to prevent saving data to file" ON)
option(USE_MPI "Enable to compile with MPI" OFF)
option(CALIBRATION "Compile the calibration executable" ON)
option(RPI "Unknown logger option" OFF)
option(SWIG_PYTHON "Compile with Swig for Python" OFF)
option(SWIG_JAVA "Compile with Swig for Java" OFF)
option(COMPILE_PLUGIN "Setting this parameter to ON will give the possibility to compile the plugins for CEINMS" OFF)
<<<<<<< HEAD
option(COMPILE_TESTS "Enable to compile tests" OFF)
=======
>>>>>>> 1c99a9e5d94ec46dea44074bfa089ee70e792b31
option(VERBOSE "Compile with extra program output" ON)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if(VERBOSE)
    add_definitions(-DVERBOSE)
endif()

# Modify the `COUT` macro
add_definitions(-DFILEANDLINE)

if(RPI)
  add_definitions(-DRPI)
endif(RPI)

if(OPENSIM4)
    add_definitions(-DOPENSIM4)
endif(OPENSIM4)

# Help making shared objects (mostly for Unix)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    # VS 2017 is throwing errors about the absence of auto_ptr without this flag
    add_definitions(-D_HAS_AUTO_PTR_ETC=1)

    # Windows cannot properly use Boost without static mode
    set(Boost_USE_STATIC_LIBS ON)

    # This is needed without the proper `Boost::` linking
    add_definitions(-DBOOST_ALL_NO_LIB)

endif()

if(WIN32)
    set(_output_dir ${CMAKE_CURRENT_SOURCE_DIR}/bin/Win/)

    add_definitions(-DWIN32)
    add_definitions(-DWINDOWS)
endif()

if(UNIX AND NOT APPLE)
    add_definitions(-DUNIX)
endif()

if(APPLE)
    add_definitions(-DAPPLE)
endif()

if(UNIX)

    set(_output_dir ${CMAKE_CURRENT_SOURCE_DIR}/bin/Unix/)

    include_directories(
        ${CMAKE_SOURCE_DIR}/build/Unix/lib/Gui
        ${CMAKE_SOURCE_DIR}/build/lib/Gui)
endif()

set(EXECUTABLE_OUTPUT_PATH ${_output_dir} CACHE PATH "Build directory")
set(LIBRARY_OUTPUT_PATH ${_output_dir} CACHE PATH "Build directory")

if(USE_MPI)
    find_package(MPI REQUIRED)
    add_definitions(-DUSE_MPI)
endif()

find_package(Boost 1.65.0 REQUIRED
        COMPONENTS timer chrono thread program_options unit_test_framework date_time filesystem system)
find_package(XSD REQUIRED)
# Note: when XercesC is installed with XSD, you might need to provide the full lib path
# with `-DXercesC_LIBRARY="[path]/xerces-c_3.lib"`
find_package(XercesC REQUIRED)

include_directories(${MPI_INCLUDE_PATH})
include_directories(
    ${CMAKE_SOURCE_DIR}/lib/Producers
    ${CMAKE_SOURCE_DIR}/lib/Producers/FromExtDevice
    ${CMAKE_SOURCE_DIR}/lib/Producers/FromFile
    ${CMAKE_SOURCE_DIR}/lib/Producers/DevicePlugin
    ${CMAKE_SOURCE_DIR}/lib/Producers/DevicePlugin/PluginHeritanceTemplate
    ${CMAKE_SOURCE_DIR}/lib/NMSmodel
    ${CMAKE_SOURCE_DIR}/lib/NMSmodel/Tendon
    ${CMAKE_SOURCE_DIR}/lib/NMSmodel/Activation
    ${CMAKE_SOURCE_DIR}/lib/Curve
    ${CMAKE_SOURCE_DIR}/lib/xmlInput
    ${CMAKE_SOURCE_DIR}/lib/FileIO
    ${CMAKE_SOURCE_DIR}/lib/FileIO/EMGgenerator
    ${CMAKE_SOURCE_DIR}/lib/OdeInt
    ${CMAKE_SOURCE_DIR}/lib/Calibration
    ${CMAKE_SOURCE_DIR}/lib/Calibration/SimAnnealParameters
    ${CMAKE_SOURCE_DIR}/lib/Calibration/ObjectiveFunctions
    ${CMAKE_SOURCE_DIR}/lib/Calibration/TorqueComputationModes
    ${CMAKE_SOURCE_DIR}/lib/Hybrid
    ${CMAKE_SOURCE_DIR}/lib/MTUSpline
    ${CMAKE_SOURCE_DIR}/lib/ModelEvaluation
    ${CMAKE_SOURCE_DIR}/lib/Gui
    ${CMAKE_SOURCE_DIR}/lib/Gui/opengl
    ${CMAKE_SOURCE_DIR}/lib/MuscleTendonScaling
    ${CMAKE_SOURCE_DIR}/lib/
    ${CMAKE_SOURCE_DIR}/XSD/
    ${CMAKE_SOURCE_DIR}/src/
)

#
# Note: We create variables for libraries that are optional to prevent
# errors later in the CMake while linking them, since the libraries themselves
# might not exist if they are disabled through options.
#

if(USE_GUI)
    add_definitions(-DUSE_GUI)

    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(GLEW REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(Qt5 REQUIRED COMPONENTS Core OpenGL Gui PrintSupport Widgets)

    # Generate Qt files automatically
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)

    set(QT_LIBRARIES
            Qt5::Widgets
            Qt5::Core
            Qt5::Gui
            Qt5::OpenGL
            Qt5::PrintSupport)

    # Include the Qt generated headers - target link is not working for some reason
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/Gui
    ${QT_INCLUDES}
	${GLEW_INCLUDE_DIRS})
    MESSAGE(STATUS ${QT_INCLUDES})

    set(GUI_LIBRARY Gui)
    set(GUICAL_LIBRARY GuiCal)
else()
    set(GUI_LIBRARY)
    set(GUICAL_LIBRARY)
endif()

if(NOT RECORD_IN_FILE)
    add_definitions(-DNO_SAVE)
endif()

if(USE_OPENSIM)

    find_package(OpenSim REQUIRED)

    add_definitions(-DUSE_OPENSIM)

    set(MuscleTendonScalingTool_LIBRARY MuscleTendonScalingTool)
    set(MTUSplineToolWrite_LIBRARY MTUSplineToolWrite)
    set(MTUSplineToolWriteMPI_LIBRARY MTUSplineToolWriteMPI)
else()
    set(MuscleTendonScalingTool_LIBRARY)
    set(MTUSplineToolWrite_LIBRARY)
    set(MTUSplineToolWriteMPI_LIBRARY)
endif()

add_subdirectory(lib/FileIO)
add_subdirectory(lib/xmlInput)
add_subdirectory(lib/NMSmodel)
add_subdirectory(lib/Producers)
add_subdirectory(lib/ModelEvaluation)
add_subdirectory(lib/tools)

if(USE_GUI)
    add_subdirectory(lib/Gui)
endif()

if(CALIBRATION)
    add_subdirectory(lib/Calibration)
endif()

if(USE_OPENSIM AND CALIBRATION)
    add_subdirectory(lib/MuscleTendonScaling)
endif()

add_subdirectory(lib/MTUSpline)
add_subdirectory(lib/tclap)
add_subdirectory(src)

if(COMPILE_PLUGIN)
    add_subdirectory(plugin)
endif()


include(Package)
