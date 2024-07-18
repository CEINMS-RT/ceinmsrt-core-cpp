cmake_minimum_required(VERSION 3.5)

project(tclap VERSION 1.2.2)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_subdirectory(include)

# Make installable

install(TARGETS tclap
        EXPORT tclapConfig
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/tclap
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

export(TARGETS tclap
        FILE "${CMAKE_CURRENT_BINARY_DIR}/tclapConfig.cmake")

install(EXPORT tclapConfig
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/tclap")

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/tclapConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY ExactVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tclapConfigVersion.cmake
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/tclap")
