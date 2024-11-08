add_executable(CEINMS
        CEINMS.cpp)

if (MSVC)
    set(windows_libs
            ws2_32
            winmm)
endif()

target_link_libraries(CEINMS
        XMLExecutionInterpreter
        NMSmodel
        FileIO
        Producers
        DynLib
		DynLibOptimization
        MTUSpline
		ModelEvaluation
        ${MTUSplineToolWrite_LIBRARY}
        ${GUI_LIBRARY}
        tclap
        XercesC::XercesC
        ${QT_LIBRARIES}
        Boost::thread
        Boost::filesystem
        Boost::timer
		XMLIKIDInterpreter
        ${windows_libs}
        qcustomplot)

if (CALIBRATION)

    add_executable(MuscleTendonScaling MuscleTendonScaling.cpp)

    target_link_libraries(MuscleTendonScaling PUBLIC
            MuscleTendonScalingTool
            XMLExecutionInterpreter
            XMLCalibrationInterpreter
            Boost::thread
            Boost::timer
            Boost::filesystem
            osimTools)

    add_executable(calibrate Calibrate.cpp)

	target_link_libraries(calibrate
		XMLCalibrationInterpreter
		NMSmodel
		MTUSpline
		XMLEMGInterpreter
		PreprocessingEMG
		SyncToolsCal
		FileIO
		${GUICAL_LIBRARY}
		${MuscleTendonScalingTool_LIBRARY}
		${MTUSplineToolWrite_LIBRARY}
		tclap
		XercesC::XercesC
		Boost::filesystem
		Boost::thread
		Boost::timer
		Boost::system
		Qt5::Widgets
		qcustomplot)

endif ()

install(TARGETS CEINMS
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Also install the XSD directory
file(GLOB XSD_FILES ${CMAKE_SOURCE_DIR}/XSD/*.xsd)

install(FILES ${XSD_FILES}
        DESTINATION ${CMAKE_INSTALL_BINDIR}/XSD)
