# Create header-only library
add_library(tclap INTERFACE)

target_include_directories(tclap INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>)
