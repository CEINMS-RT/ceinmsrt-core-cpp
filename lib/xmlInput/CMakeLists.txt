add_library(XMLNMSModelInterpreter STATIC
        ${CMAKE_SOURCE_DIR}/XSD/NMSmodel.cxx
        ExecutionXmlReader.cpp)
target_link_libraries(  XMLNMSModelInterpreter
		        XSD::XSD
		        XercesC::XercesC)


add_library(XMLExecutionInterpreter STATIC
        ${CMAKE_SOURCE_DIR}/XSD/execution.cxx
        ExecutionXmlReader.cpp)
target_link_libraries(XMLExecutionInterpreter
                XMLNMSModelInterpreter
		XSD::XSD
		XercesC::XercesC)

add_library(XMLOptimizationInterpreter
        ExecutionOptimizationXmlReader.cpp
        ${CMAKE_SOURCE_DIR}/XSD/executionOptimization.cxx)
target_link_libraries(XMLOptimizationInterpreter
		XSD::XSD
        XercesC::XercesC)

add_library(XMLEMGInterpreter
        ExecutionEmgXml.cpp
        ${CMAKE_SOURCE_DIR}/XSD/executionEMG.cxx)
target_link_libraries(XMLEMGInterpreter
		XSD::XSD
                XercesC::XercesC
                Boost::system
                Boost::filesystem)

add_library(XMLCalibrationInterpreter STATIC
                ExecutionSimulatedAnnealing.cpp
                ${CMAKE_SOURCE_DIR}/XSD/simulatedAnnealing.cxx)
        target_link_libraries(XMLCalibrationInterpreter
                XMLNMSModelInterpreter
                XMLEMGInterpreter
                XSD::XSD
                XercesC::XercesC
                )


target_compile_features(XMLEMGInterpreter PUBLIC cxx_std_17)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(XMLEMGInterpreter stdc++fs)
endif()

add_library(XMLIKIDInterpreter
        XMLInterpreter.cpp
        ${CMAKE_SOURCE_DIR}/XSD/executionIK_ID.cxx)
target_link_libraries(XMLIKIDInterpreter
		XSD::XSD
        XercesC::XercesC)
		
target_compile_features(XMLIKIDInterpreter PUBLIC cxx_std_17)
target_compile_definitions(XMLIKIDInterpreter PUBLIC -D_HAS_STD_BYTE=0)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(XMLIKIDInterpreter stdc++fs)
endif()
