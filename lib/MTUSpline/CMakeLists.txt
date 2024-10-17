add_library(MTUSpline STATIC
        MTUSpline.cpp
        MTUSplineBase.cpp
        SplineBasisFunction.cpp
        MTUSplineData.cpp
        MTUSplineDataRead.cpp
        MTUSplineInterface.cpp)

target_link_libraries(MTUSpline
        NMSmodel
        XMLExecutionInterpreter
        XercesC::XercesC
        Boost::chrono
        Boost::timer
        Boost::system)

if (USE_OPENSIM)

    add_library(MTUSplineToolWrite STATIC
            ${CMAKE_SOURCE_DIR}/lib/MTUSpline/MuscleAnalyseForSpline.cpp
            ${CMAKE_SOURCE_DIR}/lib/MTUSpline/MTUSplineDataWrite.cpp)

    target_link_libraries(MTUSplineToolWrite
            MTUSpline
            NMSmodel
            XMLExecutionInterpreter
            osimTools
            XercesC::XercesC
            Boost::system)

    if (USE_MPI)
        add_library(MTUSplineToolWriteMPI STATIC
                ${CMAKE_SOURCE_DIR}/lib/MTUSpline/MuscleAnalyseForSpline_MPI.cpp)

        target_link_libraries(MTUSplineToolWriteMPI
                MTUSplineToolWrite
                MTUSpline
                NMSmodel
                XMLExecutionInterpreter
                osimTools
                XercesC::XercesC
                Boost::system)

    endif ()
endif ()
