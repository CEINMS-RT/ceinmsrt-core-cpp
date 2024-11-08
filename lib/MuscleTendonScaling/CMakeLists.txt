if (USE_OPENSIM)
    add_library(MuscleTendonScalingTool STATIC
            ComputeMuscleTendonLength.cpp
            ComputePostureControlPoints.cpp
            OptimalFiberLengthOptimizer.cpp
            ComputeMuscleLength.cpp
            TSLAndOFLOptimizer.cpp
            ComputeTSLAndOFL.cpp
            MuscleTendonScaling.cpp
            TSLAndOFLOptimizer.cpp)

    target_link_libraries(MuscleTendonScalingTool
            NMSmodel
            XMLExecutionInterpreter
            XMLCalibrationInterpreter
            XercesC::XercesC
            Boost::system
            osimTools)
endif ()
