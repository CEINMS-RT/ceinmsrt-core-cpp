
project(TestAngleAndConsumer)

add_executable(${PROJECT_NAME} AngleAndConsumerTests.cpp)

target_link_libraries(${PROJECT_NAME}
    Boost::unit_test_framework
    Boost::system
    Boost::filesystem
    CompareFilesLib
    ResampleDataLib
    MapFromFileLib
    MapTools
    CorrelationLib
    ResampleDataLib
    )

if(UNIX)
target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE -DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}" -DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()