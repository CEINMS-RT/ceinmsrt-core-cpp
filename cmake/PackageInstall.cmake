#
# See `Package.cmake`
#
# This script will be executed when a cmake install is performed.
# Its main purpose is collecting runtime dependencies, which will
# be installed. ('Installed' in case of usage of cpack.)
#
# Note that the variables during the cmake configure _won't_ exist
# anymore. So variables to keep will have to be exported explicitly.
#

if (WINDEPLOYQT_EXECUTABLE)

    # Run `windeployqt` around the target executable and set them to install
    # CPack will pick up on this and include the files in a package

    execute_process(
            COMMAND "${CMAKE_COMMAND}" -E remove_directory "${QTDeploy_DIR}"
            COMMAND "${CMAKE_COMMAND}" -E
            env PATH="${QTBin_DIR}" "${WINDEPLOYQT_EXECUTABLE}"
            --verbose 0
            --no-compiler-runtime
            --no-angle
            --no-webkit2
            --no-quick-import
            --no-translations
            --dir "${QTDeploy_DIR}" "${TARGET_CEINMS_FILE}")

    file(GLOB QTDeploy_FILES ${QTDeploy_DIR}/*)

    foreach (_file ${QTDeploy_FILES})

        file(INSTALL
                DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
                TYPE SHARED_LIBRARY
                FOLLOW_SYMLINK_CHAIN
                FILES "${_file}")

    endforeach ()

endif ()

if (TARGET_CEINMS_FILE)

    file(GET_RUNTIME_DEPENDENCIES
            RESOLVED_DEPENDENCIES_VAR _r_deps
            UNRESOLVED_DEPENDENCIES_VAR _u_deps
            CONFLICTING_DEPENDENCIES_PREFIX CONFLICTING_DEPENDENCIES
            EXECUTABLES ${TARGET_CEINMS_FILE}
            DIRECTORIES ${MY_DEPENDENCY_PATHS}
            PRE_EXCLUDE_REGEXES "api-ms-" "ext-ms-" "qt5"
            POST_EXCLUDE_REGEXES "(.*)system32(.*)")

    foreach (_file ${_r_deps})

        # `install()` doesn't work here
        file(INSTALL
                DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
                TYPE SHARED_LIBRARY
                FOLLOW_SYMLINK_CHAIN
                FILES "${_file}")

    endforeach ()

    list(LENGTH _u_deps _u_length)
    if ("${_u_length}" GREATER 0)
        message(WARNING "Unresolved dependencies detected during install or pack! ${_u_deps}")
    endif ()
else()
    message(WARNING "Target executable was not defined!")
endif()
