#
# This script will enable use to package our application into e.g. a zip or an installer.
# CPack is going to pack all the targets and files marked with `install()`.
#
# Note: setting `cmake_policy(SET CMP0087 NEW)` in the project CMakeLists.txt is required.
#
# The tricky part is packing dependencies too. This is done with `GET_RUNTIME_DEPENDENCIES`
# windeployqt.exe, in installed codes.
#

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "CEINMS-RT Platform")
SET(CPACK_PACKAGE_VENDOR "BE-ET")

include(InstallRequiredSystemLibraries)
include(CPack)

if (CMAKE_SYSTEM_NAME MATCHES "Windows")

    # On Windows, use `windeployqt` to create a set of runtime libraries
    # Find the Qt bin_dir
    get_target_property(_qmake_executable Qt5::qmake IMPORTED_LOCATION)
    get_filename_component(QTBin_DIR "${_qmake_executable}" DIRECTORY)

    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${QTBin_DIR}")

    # Export variables to the install-time script
    install(CODE "set(QTDeploy_DIR \"${CMAKE_CURRENT_BINARY_DIR}/qtDeploy/\")")
    install(CODE "set(QTBin_DIR \"${QTBin_DIR}\")")
    install(CODE "set(WINDEPLOYQT_EXECUTABLE \"${WINDEPLOYQT_EXECUTABLE}\")")

    # The TARGET_FILE generator won't work from the install script, so execute it now
    install(CODE "set(TARGET_CEINMS_FILE \"$<TARGET_FILE:CEINMS>\")")

else ()

    # The TARGET_FILE generator seems to fail on Linux
    install(CODE "set(TARGET_CEINMS_FILE \"${EXECUTABLE_OUTPUT_PATH}/CEINMS\")")

endif ()

set(MY_DEPENDENCY_PATHS
        ${PACKAGE_DEPENDENCIES_DIRECTORIES}
        ${OpenSim_BIN_DIR})

if (XSD_EXECUTABLE)
    get_filename_component(_xsd_bin_dir ${XSD_EXECUTABLE} DIRECTORY)
    list(APPEND MY_DEPENDENCY_PATHS ${_xsd_bin_dir})
endif ()

if (GLEW_INCLUDE_DIR)
    get_filename_component(_glew_root ${GLEW_INCLUDE_DIR} DIRECTORY)
    list(APPEND MY_DEPENDENCY_PATHS "${_glew_root}/bin/Release/x64")
endif ()

# Transfer the value of ${MY_DEPENDENCY_PATHS} into the install script
# These dirs are needed, for some reason the standard search paths don't work here
install(CODE "set(MY_DEPENDENCY_PATHS \"${MY_DEPENDENCY_PATHS}\")")

# Place install script
install(SCRIPT "cmake/PackageInstall.cmake")
