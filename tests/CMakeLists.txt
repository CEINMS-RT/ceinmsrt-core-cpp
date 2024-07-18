
# project("MasterTestSuite")

# add_executable(${PROJECT_NAME} masterTestSuite.cpp)

# target_link_libraries(${PROJECT_NAME}
# 	Boost::unit_test_framework
#     ProducerVirtualTests)

execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/data
)

# set_target_properties(${PROJECT_NAME}
#     PROPERTIES
#     ARCHIVE_OUTPUT_DIRECTORY ${_output_dir}
#     LIBRARY_OUTPUT_DIRECTORY ${_output_dir}
#     RUNTIME_OUTPUT_DIRECTORY ${_output_dir}
# )


if(NOT EXISTS ${CMAKE_SOURCE_DIR}/data/testData)
message("Collecting test data. Please wait.")
execute_process(
    COMMAND git clone git@bitbucket.org:ctw-bw/ceinms-test-data.git ${CMAKE_SOURCE_DIR}/data/testData
)
else()
message("Test data folder already exists. Please delete it if you want to re-download it.")
endif()

add_subdirectory(calibrationTests)
add_subdirectory(coreTests)
add_subdirectory(lib)
add_subdirectory(pluginInterfaceTests)