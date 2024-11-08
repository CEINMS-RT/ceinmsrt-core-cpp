#
# No official FindXSD scritp is available.
#
# Attempt to find the xsd application in various places. If found, the full
# path will be in XSD_EXECUTABLE. Look in the usual locations, as well as in
# the 'bin' directory in the path given in the XSD_ROOT environment variable.
# Will also find and set XSD_INCLUDE_DIR. The find command will fail if the
# include was not found.
#
# Will import the `XSD::XSD` header-only target and the `XSD_SCHEMA` macro.
#
# As a bonus this script will append the library directory of XercesC to the
# search, since on Windows XercesC is installed together with XSD.
#

find_program(XSD_EXECUTABLE
        NAMES
            xsd
            xsdcxx
        HINTS
            ${RWSL_DEPS}/xsd/bin
            ${XSD_ROOT}/bin
            $ENV{XSD_ROOT}/bin
            "C:/Program Files\ (x86)/CodeSynthesis\ XSD\ 4.0/bin"
        ENV PATH
        DOC "XSD executable path")
mark_as_advanced(XSD_EXECUTABLE)

find_path(XSD_INCLUDE_DIR
        NAMES "xsd/cxx/config.hxx"
        DOC "XSD C++ include directory")
mark_as_advanced(XSD_INCLUDE_DIR)

#
# General CMake package configuration.

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XSD
        REQUIRED_VARS XSD_INCLUDE_DIR)

if(XSD_FOUND)
    set(XSD_INCLUDE_DIRS "${XSD_INCLUDE_DIR}")

    # For header-only libraries
    if(NOT TARGET XSD::XSD)
        add_library(XSD::XSD INTERFACE IMPORTED)
        if(XSD_INCLUDE_DIRS)
            set_target_properties(XSD::XSD PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${XSD_INCLUDE_DIRS}")
        endif()
    endif()

    # Create a hint for finding XercesC later
    if(NOT XercesC_LIBRARY)
        get_filename_component(_xsd_root ${XSD_INCLUDE_DIR} DIRECTORY)
        set(_xerces_lib_dir "${_xsd_root}/lib64/vc-12.0")

        if(EXISTS ${_xerces_lib_dir})
            # Add to search paths
            list(APPEND CMAKE_PREFIX_PATH "${_xerces_lib_dir}")
        endif()
    endif()
endif()

# 
# Macro that attempts to generate C++ files from an XML schema. The NAME
# argument is the name of the CMake variable to use to store paths to the
# derived C++ source file. The FILE argument is the path of the schema file to
# process. Additional arguments should be XSD command-line options.
#
# Example:
#
# XSD_SCHEMA( FOO_SRCS Foo.xsd --root-element-first --generate-serialization )
#
# On return, FOO_SRCS will contain Foo.cxx.

MACRO(XSD_SCHEMA NAME FILE)

    #
    # Make a full path from the source directory

    set(xs_SRC "${FILE}")

    #
    # XSD will generate two or three C++ files (*.cxx,*.hxx,*.ixx). Get the
    # destination file path sans any extension and then build paths to the
    # generated files.

    get_filename_component(xs_FILE "${FILE}" NAME_WE)
    set(xs_CXX "${CMAKE_CURRENT_BINARY_DIR}/${xs_FILE}.cxx")
    set(xs_HXX "${CMAKE_CURRENT_BINARY_DIR}/${xs_FILE}.hxx")
    set(xs_IXX "${CMAKE_CURRENT_BINARY_DIR}/${xs_FILE}.ixx")

    #
    # Add the source files to the NAME variable, which presumably will be used to
    # define the source of another target.

    list(APPEND ${NAME} ${xs_CXX})

    #
    # Set up a generator for the output files from the given schema file using
    # the XSD cxx-tree command.

    add_custom_command(OUTPUT "${xs_CXX}" "${xs_HXX}" "${xs_IXX}"
            COMMAND ${XSD_EXECUTABLE}
            ARGS "cxx-tree" ${ARGN} ${xs_SRC}
            DEPENDS ${xs_SRC})

    #
    # Don't fail if a generated file does not exist.

    set_source_files_properties("${xs_CXX}" "${xs_HXX}" "${xs_IXX}"
            PROPERTIES GENERATED TRUE)

ENDMACRO(XSD_SCHEMA)
